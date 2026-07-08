#!/usr/bin/env bash
# 准备 libtorch 运行时库名兼容软链接。
# 部分 GPU 版 libtorch 会把 CUDA 库打包成带哈希的文件名，但库自身 SONAME
# 仍是标准名。直接链接 C++ 程序时，运行期可能按 SONAME 查找标准文件名。
set -euo pipefail

LIBTORCH_ROOT="${1:-${LIBTORCH_ROOT:-}}"

if [ -z "$LIBTORCH_ROOT" ]; then
  for candidate in \
    "$PWD/third_party/libtorch" \
    "$HOME/third_party/libtorch" \
    "/home/xbh/third_party/libtorch" \
    "/home/ros/Guoyulun/Download/libtorch"; do
    if [ -f "$candidate/share/cmake/Torch/TorchConfig.cmake" ]; then
      LIBTORCH_ROOT="$candidate"
      break
    fi
  done
fi

if [ -z "$LIBTORCH_ROOT" ]; then
  echo "未找到 libtorch，请设置 LIBTORCH_ROOT=/path/to/libtorch。" >&2
  exit 1
fi

if [ ! -d "$LIBTORCH_ROOT" ]; then
  echo "libtorch 根目录不存在：$LIBTORCH_ROOT" >&2
  exit 1
fi

LIBTORCH_ROOT="$(cd "$LIBTORCH_ROOT" && pwd)"
LIBTORCH_LIB_DIR="$LIBTORCH_ROOT/lib"

if [ ! -d "$LIBTORCH_LIB_DIR" ]; then
  echo "libtorch 库目录不存在：$LIBTORCH_LIB_DIR" >&2
  exit 1
fi

if ! command -v readelf >/dev/null 2>&1; then
  echo "缺少 readelf，请安装 binutils 后重试。" >&2
  exit 1
fi

created_count=0
checked_count=0
shopt -s nullglob

for library_path in "$LIBTORCH_LIB_DIR"/lib*.so*; do
  if [ -L "$library_path" ] || [ ! -f "$library_path" ]; then
    continue
  fi

  soname="$(readelf -d "$library_path" 2>/dev/null | sed -n 's/.*Library soname: \[\(.*\)\].*/\1/p' | head -n 1)"
  if [ -z "$soname" ]; then
    continue
  fi

  checked_count=$((checked_count + 1))
  library_name="$(basename "$library_path")"
  if [ "$library_name" = "$soname" ]; then
    continue
  fi

  link_path="$LIBTORCH_LIB_DIR/$soname"
  if [ ! -e "$link_path" ] && [ ! -L "$link_path" ]; then
    ln -s "$library_name" "$link_path"
    created_count=$((created_count + 1))
    echo "已创建 libtorch 兼容软链接：$soname -> $library_name"
    continue
  fi

  if [ -L "$link_path" ]; then
    current_target="$(readlink "$link_path")"
    current_realpath="$(readlink -f "$link_path" 2>/dev/null || true)"
    library_realpath="$(readlink -f "$library_path")"
    if [ "$current_target" = "$library_name" ] || [ "$current_realpath" = "$library_realpath" ]; then
      continue
    fi
  fi

  echo "发现已存在的库名 $link_path，未覆盖。请确认它是否与 $library_name 兼容。" >&2
  exit 1
done

echo "libtorch 运行库检查完成：$LIBTORCH_LIB_DIR，检查 $checked_count 个共享库，新建 $created_count 个软链接。"
