#!/usr/bin/env python3
"""将本目录受控格式的 Markdown 文档导出为中文 PDF。"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parent
BUILD = ROOT / ".pdf-build"
DOCUMENTS = [
    ROOT / "01-技术方案报告.md",
    ROOT / "02-自测试报告.md",
    ROOT / "03-使用说明.md",
    ROOT / "04-项目论文.md",
]


PREAMBLE = r"""\documentclass[UTF8,fontset=none,a4paper,12pt]{ctexart}
\usepackage[margin=2.3cm,headheight=15pt]{geometry}
\usepackage{fontspec}
\usepackage{graphicx}
\usepackage{longtable}
\usepackage{array}
\usepackage{booktabs}
\usepackage{amsmath,amssymb}
\usepackage{xcolor}
\usepackage{hyperref}
\usepackage{fancyhdr}
\usepackage{enumitem}
\usepackage{fvextra}
\usepackage{needspace}
\setCJKmainfont{Noto Serif CJK SC}
\setCJKsansfont{Noto Sans CJK SC}
\setCJKmonofont{Noto Sans Mono CJK SC}
\setCJKfamilyfont{zhkai}{Noto Serif CJK SC}
\setmainfont{DejaVu Serif}
\setsansfont{DejaVu Sans}
\setmonofont{Noto Sans Mono CJK SC}
\hypersetup{colorlinks=true,linkcolor=black,urlcolor=blue,pdfborder={0 0 0}}
\Urlmuskip=0mu plus 1mu
\setlength{\parindent}{2em}
\setlength{\parskip}{0.35em}
\linespread{1.28}
\emergencystretch=4em
\sloppy
\raggedbottom
\setlist{nosep,leftmargin=2.2em}
\pagestyle{fancy}
\fancyhf{}
\fancyhead[L]{DG-202602 项目文档}
\fancyhead[R]{\leftmark}
\fancyfoot[C]{\thepage}
\DefineVerbatimEnvironment{DocCode}{Verbatim}{breaklines=true,breakanywhere=true,fontsize=\small,frame=single,framesep=3mm}
\newenvironment{DocQuote}{\begin{quote}\small\color{black!75}}{\end{quote}}
\renewcommand{\contentsname}{目录}
\renewcommand{\arraystretch}{1.32}
"""


def escape(text: str) -> str:
    table = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(table.get(char, char) for char in text)


TOKEN = re.compile(
    r"(\\\(.*?\\\)|`[^`]+`|\*\*.*?\*\*|!\[[^]]*\]\([^)]+\)|\[[^]]+\]\([^)]+\))"
)


def inline(text: str) -> str:
    result: list[str] = []
    for part in TOKEN.split(text):
        if not part:
            continue
        if part.startswith(r"\(") and part.endswith(r"\)"):
            result.append(part)
        elif part.startswith("`"):
            code = part[1:-1]
            if any(char in code for char in "/_."):
                result.append(r"\nolinkurl{" + code.replace("}", r"\}") + "}")
            else:
                result.append(r"\texttt{" + escape(code) + "}")
        elif part.startswith("**"):
            result.append(r"\textbf{" + inline(part[2:-2]) + "}")
        elif part.startswith("!["):
            match = re.match(r"!\[([^]]*)\]\(([^)]+)\)", part)
            result.append(escape(match.group(1)) if match else escape(part))
        elif part.startswith("["):
            match = re.match(r"\[([^]]+)\]\(([^)]+)\)", part)
            if match:
                result.append(r"\href{" + escape(match.group(2)) + "}{" + inline(match.group(1)) + "}")
            else:
                result.append(escape(part))
        else:
            result.append(escape(part))
    return "".join(result)


def heading_inline(text: str) -> str:
    """标题只保留可安全写入 PDF 书签的纯文本格式。"""
    text = re.sub(r"`([^`]+)`", r"\1", text)
    text = re.sub(r"\*\*(.*?)\*\*", r"\1", text)
    return escape(text)


def table_to_latex(lines: list[str]) -> str:
    rows = [[cell.strip() for cell in line.strip().strip("|").split("|")] for line in lines]
    if len(rows) >= 2 and all(re.fullmatch(r":?-{3,}:?", cell) for cell in rows[1]):
        rows.pop(1)
    count = max(len(row) for row in rows)
    width = max(0.045, (0.94 - 0.027 * count) / count)
    columns = "|" + "|".join([f">{{\\raggedright\\arraybackslash}}p{{{width:.3f}\\textwidth}}"] * count) + "|"
    size = r"\scriptsize" if count >= 7 else r"\footnotesize" if count >= 5 else r"\small"
    output = [r"\Needspace{10\baselineskip}", size, r"\begin{longtable}{" + columns + "}", r"\hline"]
    for index, row in enumerate(rows):
        padded = row + [""] * (count - len(row))
        cells = [inline(cell) for cell in padded]
        if index == 0:
            cells = [r"\textbf{" + cell + "}" for cell in cells]
        output.append(" & ".join(cells) + r" \\ \hline")
    output.append(r"\end{longtable}")
    output.append(r"\normalsize")
    return "\n".join(output)


def markdown_to_latex(path: Path) -> tuple[str, str, str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    title = path.stem
    body: list[str] = []
    cover: list[str] = []
    paragraph: list[str] = []
    list_kind: str | None = None
    index = 0

    if lines and lines[0].startswith("# "):
        title = lines[0][2:].strip()
        index = 1
        while index < len(lines) and lines[index].strip() != "---":
            if lines[index].strip():
                cover.append(inline(lines[index].strip()))
            index += 1
        if index < len(lines) and lines[index].strip() == "---":
            index += 1

    def flush_paragraph() -> None:
        if paragraph:
            body.append(inline(" ".join(item.strip() for item in paragraph)))
            body.append("")
            paragraph.clear()

    def close_list() -> None:
        nonlocal list_kind
        if list_kind:
            body.append(r"\end{" + list_kind + "}")
            body.append("")
            list_kind = None

    while index < len(lines):
        line = lines[index]
        stripped = line.strip()

        if line.startswith("```"):
            flush_paragraph()
            close_list()
            code: list[str] = []
            index += 1
            while index < len(lines) and not lines[index].startswith("```"):
                code.append(lines[index])
                index += 1
            body.extend([r"\begin{DocCode}", *code, r"\end{DocCode}", ""])
        elif stripped == r"\[":
            flush_paragraph()
            close_list()
            math_lines = [r"\["]
            index += 1
            while index < len(lines):
                math_lines.append(lines[index])
                if lines[index].strip() == r"\]":
                    break
                index += 1
            body.extend(math_lines + [""])
        elif re.match(r"^\|.*\|$", stripped):
            flush_paragraph()
            close_list()
            table_lines: list[str] = []
            while index < len(lines) and re.match(r"^\|.*\|$", lines[index].strip()):
                table_lines.append(lines[index])
                index += 1
            body.extend([table_to_latex(table_lines), ""])
            continue
        elif re.match(r"^!\[[^]]*\]\([^)]+\)$", stripped):
            flush_paragraph()
            close_list()
            match = re.match(r"^!\[([^]]*)\]\(([^)]+)\)$", stripped)
            caption, image_path = match.groups()
            body.extend([
                r"\begin{figure}[htbp]",
                r"\centering",
                r"\includegraphics[width=0.96\textwidth]{\detokenize{" + image_path + "}}",
                r"\caption{" + inline(caption) + "}",
                r"\end{figure}",
                "",
            ])
        elif stripped.startswith(">"):
            flush_paragraph()
            close_list()
            quote_lines: list[str] = []
            while index < len(lines) and lines[index].strip().startswith(">"):
                quote_lines.append(lines[index].strip()[1:].strip())
                index += 1
            body.extend([r"\begin{DocQuote}", inline(" ".join(quote_lines)), r"\end{DocQuote}", ""])
            continue
        elif re.match(r"^#{1,4} ", line):
            flush_paragraph()
            close_list()
            level = len(line) - len(line.lstrip("#"))
            heading = line[level + 1 :].strip()
            heading = re.sub(r"^\d+(?:\.\d+)*\s+", "", heading)
            unnumbered = heading in {"摘要", "Abstract", "参考资料", "参考文献"} or heading.startswith("附录")
            if level == 1 and title == path.stem:
                title = heading
            elif level == 1:
                body.append(r"\section*{" + heading_inline(heading) + "}")
            elif level == 2:
                if unnumbered:
                    body.append(r"\section*{" + heading_inline(heading) + "}")
                    body.append(r"\addcontentsline{toc}{section}{" + heading_inline(heading) + "}")
                else:
                    body.append(r"\section{" + heading_inline(heading) + "}")
            elif level == 3:
                body.append(r"\subsection{" + heading_inline(heading) + "}")
            else:
                body.append(r"\subsubsection{" + heading_inline(heading) + "}")
            body.append("")
        elif re.match(r"^[-*] ", stripped) or re.match(r"^\d+\. ", stripped):
            flush_paragraph()
            requested = "enumerate" if re.match(r"^\d+\. ", stripped) else "itemize"
            if list_kind != requested:
                close_list()
                body.append(r"\begin{" + requested + "}")
                list_kind = requested
            content = re.sub(r"^(?:[-*]|\d+\.)\s+", "", stripped)
            if content.startswith("[ ] "):
                content = "□ " + content[4:]
            elif content.startswith("[x] ") or content.startswith("[X] "):
                content = "■ " + content[4:]
            body.append(r"\item " + inline(content))
        elif not stripped:
            flush_paragraph()
            close_list()
        elif stripped == "---":
            flush_paragraph()
            close_list()
            body.append(r"\vspace{0.4em}\hrule\vspace{0.8em}")
        else:
            if list_kind:
                close_list()
            paragraph.append(stripped)
            if line.endswith("  "):
                flush_paragraph()
        index += 1

    flush_paragraph()
    close_list()
    return title, r"\\[0.8em]".join(cover), "\n".join(body)


def export(path: Path) -> Path:
    title, cover, body = markdown_to_latex(path)
    tex_path = BUILD / f"{path.stem}.tex"
    for suffix in (".aux", ".log", ".out", ".toc"):
        (BUILD / f"{path.stem}{suffix}").unlink(missing_ok=True)
    tex = (
        PREAMBLE
        + "\n"
        + r"\begin{document}" + "\n"
        + r"\begin{titlepage}" + "\n"
        + r"\centering" + "\n"
        + r"\vspace*{4cm}" + "\n"
        + r"{\Huge\bfseries " + heading_inline(title) + r"\par}" + "\n"
        + r"\vfill" + "\n"
        + r"{\large " + cover + r"\par}" + "\n"
        + r"\end{titlepage}" + "\n"
        + r"\tableofcontents" + "\n"
        + r"\newpage" + "\n"
        + body
        + "\n"
        + r"\end{document}" + "\n"
    )
    tex_path.write_text(tex, encoding="utf-8")
    command = [
        "xelatex",
        "-interaction=nonstopmode",
        "-halt-on-error",
        f"-output-directory={BUILD}",
        str(tex_path),
    ]
    for _ in range(2):
        subprocess.run(command, cwd=ROOT, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    pdf_source = BUILD / f"{path.stem}.pdf"
    pdf_target = ROOT / f"{path.stem}.pdf"
    shutil.copy2(pdf_source, pdf_target)
    return pdf_target


def main() -> None:
    BUILD.mkdir(exist_ok=True)
    for document in DOCUMENTS:
        target = export(document)
        print(f"已导出：{target.name}")


if __name__ == "__main__":
    main()
