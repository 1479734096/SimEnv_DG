#!/usr/bin/env python3
import json
import csv
from pathlib import Path
import sys

def analyze_failures(csv_path: Path):
    if not csv_path.exists():
        return "没有找到汇总数据 (summary.csv)，无法分析失败原因。"

    total_runs = 0
    timeout_runs = 0
    high_far_runs = 0
    low_rec_runs = 0
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            total_runs += 1
            if float(row['exploration_time']) >= float(row['timeout']) - 5.0:
                timeout_runs += 1
            
            # Remove '%' and handle missing values
            far_str = row['false_alarm_rate'].replace('%', '') if 'false_alarm_rate' in row and row['false_alarm_rate'] else '0'
            rec_str = row['recognition_rate'].replace('%', '') if 'recognition_rate' in row and row['recognition_rate'] else '0'
            
            try:
                far = float(far_str) / 100.0
            except ValueError:
                far = 0.0
                
            try:
                rec = float(rec_str) / 100.0
            except ValueError:
                rec = 0.0
                
            if far > 0.1:
                high_far_runs += 1
            if rec < 0.6:
                low_rec_runs += 1

    if total_runs == 0:
        return "汇总数据中没有运行记录。"

    analysis = f"在总计 {total_runs} 次测试中：\n"
    analysis += f"- 探索超时比例: {timeout_runs/total_runs:.1%} ({timeout_runs}/{total_runs})\n"
    analysis += f"- 虚警率(>10%)偏高比例: {high_far_runs/total_runs:.1%} ({high_far_runs}/{total_runs})\n"
    analysis += f"- 识别率(<60%)偏低比例: {low_rec_runs/total_runs:.1%} ({low_rec_runs}/{total_runs})\n\n"

    analysis += "### 优化建议\n"
    if timeout_runs / total_runs > 0.3:
        analysis += "- **探索超时偏高**：建议增大 `controller/max_linear_vel` 或 `controller/max_angular_vel`，或减少等待和扫描时间 (`no_frontier_scan_duration`)。\n"
    if high_far_runs / total_runs > 0.3:
        analysis += "- **虚警率偏高**：建议增加多帧确认次数 `tracker/min_hits`，或增大 NMS 过滤半径 `tracker/nms_radius`，或者提高置信度阈值 `detection/min_confidence`。\n"
    if low_rec_runs / total_runs > 0.3:
        analysis += "- **识别率偏低**：建议降低目标颜色或圆度的阈值要求，或者减小 `tracker/min_hits`，但需注意可能增加虚警率。\n"

    if timeout_runs == 0 and high_far_runs == 0 and low_rec_runs == 0:
        analysis += "- 当前参数表现良好，未发现明显瓶颈。\n"

    return analysis

def main():
    workspace_dir = Path("/home/xbh/tzb26/codeDG/SimEnv")
    results_dir = workspace_dir / "results"
    tune_dir = results_dir / "tuning"
    report_json_path = tune_dir / "tuning_report.json"
    csv_path = results_dir / "summary.csv"
    
    report_md_path = results_dir / "optimization_report.md"

    md_content = "# danger_search 参数优化报告\n\n"

    if report_json_path.exists():
        with open(report_json_path, 'r') as f:
            report_data = json.load(f)
            
        md_content += f"## 最佳参数组合\n\n"
        md_content += f"**综合得分**: {report_data.get('best_score', 0):.2f}\n\n"
        md_content += "```yaml\n"
        for k, v in report_data.get('best_params', {}).items():
            md_content += f"{k}: {v}\n"
        md_content += "```\n\n"
        
        md_content += "## 候选参数表现\n\n"
        md_content += "| 参数组合 | 平均分 | 最差分 |\n"
        md_content += "|---|---|---|\n"
        for res in report_data.get('all_results', []):
            params_str = ", ".join([f"{k.split('/')[-1]}={v}" for k, v in res['params'].items()])
            md_content += f"| `{params_str}` | {res['avg_score']:.2f} | {res['worst_score']:.2f} |\n"
        md_content += "\n"
    else:
        md_content += "未找到 `tuning_report.json`，暂无多组参数对比结果。\n\n"

    md_content += "## 失败归因与优化方向\n\n"
    md_content += analyze_failures(csv_path)

    with open(report_md_path, 'w') as f:
        f.write(md_content)

    print(f"优化报告已生成至 {report_md_path}")

if __name__ == "__main__":
    main()
