#!/bin/bash
# 构建 GitLab/GitHub Pages 的统一脚本 - 已适配多级目录清晰隔离版
# 依赖: pandoc, sed, awk
# 用法: bash scripts/build-pages.sh

set -e

OUTPUT_DIR="${1:-public}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSS_SRC="$SCRIPT_DIR/style.css"
DOC_SRC="$(cd "$SCRIPT_DIR/../doc" && pwd)"

echo "[build-pages] Output dir: $OUTPUT_DIR"
echo "[build-pages] CSS source: $CSS_SRC"
echo "[build-pages] Doc source: $DOC_SRC"

# 1. 创建输出目录
mkdir -p "$OUTPUT_DIR/css"
mkdir -p "$OUTPUT_DIR/doc"

# 2. 复制 CSS 文件到产物目录
if [ -f "$CSS_SRC" ]; then
    cp "$CSS_SRC" "$OUTPUT_DIR/css/style.css"
else
    echo "[ERROR] CSS file not found at $CSS_SRC"
    exit 1
fi

# 3. 转换根目录 README.md (项目首页) 为首页 index.html
if [ -f "README.md" ]; then
    pandoc README.md -f markdown -t html -s \
        --metadata title="visionAlgorithm" \
        -c css/style.css \
        -o "$OUTPUT_DIR/index.html"
else
    echo "[WARNING] Root README.md not found! Creating a default index.html"
    echo "<html><head><link rel='stylesheet' href='css/style.css'></head><body><h1>visionAlgorithm</h1></body></html>" > "$OUTPUT_DIR/index.html"
fi

# ========================================================
# 4. 递归转换并收集多级目录结构
# ========================================================
echo "[build-pages] Converting markdown and identifying categories..."
RAW_ENTRIES=""

if [ -d "$DOC_SRC" ]; then
    # 使用 find 深度为 2, 正好抓取 doc/ 目录下的第一层子文件夹（common, detector）
    find "$DOC_SRC" -mindepth 2 -maxdepth 2 -name '*.md' -type f | while read -r md_file; do
        base_name=$(basename "$md_file" .md)
        
        # 提取文件所在的父目录名称（如 common 或 detector）并转为大写作为展示区隔
        dir_name=$(basename "$(dirname "$md_file")")
        category=$(echo "$dir_name" | tr '[:lower:]' '[:upper:]')

        # 统一扁平化输出到 public/doc/ 下, 确保样式和链接路径不被打乱
        pandoc "$md_file" -s \
            --metadata title="${base_name}" \
            -c ../css/style.css \
            -o "$OUTPUT_DIR/doc/${base_name}.html"

        # 排除导航用的 README, 将其余文件和对应分类存入临时变量
        if [ "$base_name" != "README" ]; then
            echo "${category}:${base_name}" >> .tmp_entries
        fi
    done
else
    echo "[WARNING] doc/ directory not found at $DOC_SRC"
fi

# 读取并排序捕获到的目录项（保证相同分类的文件聚集在一起）
if [ -f .tmp_entries ]; then
    RAW_ENTRIES=$(sort .tmp_entries)
    rm -f .tmp_entries
fi

# 5. 修复 markdown 内部链接(.md)为 .html
echo "[build-pages] Fixing internal markdown links..."
find "$OUTPUT_DIR" -name '*.html' -type f | while read -r html_path; do
    sed -i 's/\(href="[^"]*\)\.md"/\1.html"/g' "$html_path"
done

# ========================================================
# 6. 遍历所有 HTML 文件, 动态注入带目录隔离的侧边栏
# ========================================================
echo "[build-pages] Injecting segmented sidebar into HTML files..."
find "$OUTPUT_DIR" -name '*.html' -type f | while read -r html_file; do
    # 判断是首页还是 doc 页面, 决定链接前缀和首页 href
    case "$html_file" in
        */index.html)
            link_prefix="doc/"
            home_href="index.html"
            ;;
        *)
            link_prefix=""
            home_href="../index.html"
            ;;
    esac

    base_name=$(basename "$html_file" .html)

    # 基础结构
    build_sidebar="
<input type=\"checkbox\" id=\"sidebar-toggler\" class=\"sidebar-toggler-input\">
<nav class=\"sidebar\" id=\"sidebar\">
<div class=\"sidebar-header\"><a href=\"${home_href}\">🏠 首页</a></div>
<ul>"

    # 文档首页 README 置顶
    if [ "$base_name" = "README" ]; then
        build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}README.html\" class=\"active\">📖 文档首页</a></li>"
    else
        build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}README.html\">📖 文档首页</a></li>"
    fi

    # 核心：解析 RAW_ENTRIES 并为不同文件夹注入“大写分类标题”
    prev_category=""
    while IFS=: read -r cat name; do
        [ -z "$cat" ] || [ -z "$name" ] && continue
        
        # 如果切换了新文件夹, 插入一行分类分割线
        if [ "$cat" != "$prev_category" ]; then
            build_sidebar="${build_sidebar}
<li class=\"sidebar-section\" style=\"margin-top: 12px; font-weight: bold; color: var(--text-secondary, #57606a); border-bottom: 1px dashed #d0d7de; padding-bottom: 2px;\">📁 ${cat}</li>"
            prev_category="$cat"
        fi

        # 插入文件链接并检测高亮
        if [ "$base_name" = "$name" ]; then
            build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}${name}.html\" class=\"active\" style=\"padding-left: 20px;\">📄 ${name}</a></li>"
        else
            build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}${name}.html\" style=\"padding-left: 20px;\">📄 ${name}</a></li>"
        fi
    done <<< "$RAW_ENTRIES"

    build_sidebar="${build_sidebar}
</ul>
</nav>
<label for=\"sidebar-toggler\" class=\"sidebar-toggle\">☰</label>
<div class=\"page-content\">
"

    # 使用 awk 将拼接好的侧边栏注入到 HTML 的 <body> 中
    awk -v sidebar="$build_sidebar" '
    {
        if ($0 ~ /<body[^>]*>/) {
            print
            print sidebar
        } else if ($0 ~ /<\/body>/) {
            print "</div>"
            print
        } else {
            print
        }
    }
    ' "$html_file" > "${html_file}.tmp" && mv "${html_file}.tmp" "$html_file"

    echo "[OK] Injected segmented sidebar into: $html_file"
done

echo "[build-pages] Done! Output in: $OUTPUT_DIR"