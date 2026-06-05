#!/bin/bash
# 构建 GitHub Pages 的脚本 - 已适配多级子目录
# 触发环境: 运行于 gh-pages 分支的 GitHub Actions 中
# 依赖: pandoc, sed, awk, grep
# 用法: bash scripts/build-pages-gh.sh

set -e

OUTPUT_DIR="${1:-public}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSS_SRC="$SCRIPT_DIR/style-gh.css"
DOC_SRC="$(cd "$SCRIPT_DIR/../doc" && pwd)"

echo "[build-pages-gh] Output dir: $OUTPUT_DIR"
echo "[build-pages-gh] CSS source: $CSS_SRC"
echo "[build-pages-gh] Doc source: $DOC_SRC"

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
        --metadata pagetitle="visionAlgorithm" \
        -o "$OUTPUT_DIR/index.html"
else
    echo "[WARNING] Root README.md not found! Creating a default index.html"
    echo "<html><head><link rel='stylesheet' href='css/style.css'></head><body><h1>visionAlgorithm</h1></body></html>" > "$OUTPUT_DIR/index.html"
fi

# 4. 递归转换 doc/ 及其子目录下所有 markdown 文件为 HTML
echo "[build-pages-gh] Converting markdown to HTML (recursive)..."
if [ -d "$DOC_SRC" ]; then
    # 使用 find 查找所有子目录下的 .md 文件
    find "$DOC_SRC" -name '*.md' -type f | while read -r md_file; do
        base_name=$(basename "$md_file" .md)
        
        # 跳过用作目录导航的 README.md
        [ "$base_name" = "README" ] && continue

        # 将所有 HTML 统一扁平化输出到 public/doc/ 下，确保 CSS 和相对链接不打破
        pandoc "$md_file" -s \
            --metadata title="${base_name}" \
            -c ../css/style.css \
            -o "$OUTPUT_DIR/doc/${base_name}.html"
    done
else
    echo "[WARNING] doc/ directory not found at $DOC_SRC"
fi

# 5. 修复 markdown 内部链接(.md)为 .html
echo "[build-pages-gh] Fixing internal markdown links..."
find "$OUTPUT_DIR" -name '*.html' -type f | while read -r html_path; do
    sed -i 's/\(href="[^"]*\)\.md"/\1.html"/g' "$html_path"
done

# 6. 自动寻找 doc/ 目录下的 README.md 并提取树状目录结构
echo "[build-pages-gh] Building tree structure from doc/**/README.md..."
# 自动定位新的 README.md 位置
NAV_README=$(find "$DOC_SRC" -name 'README.md' -type f | head -n 1)

build_tree_entries() {
    current_section=""
    entries=""
    if [ -n "$NAV_README" ] && [ -f "$NAV_README" ]; then
        while IFS= read -r line || [ -n "$line" ]; do
            case "$line" in
                "## "*)
                    current_section=$(echo "$line" | sed 's/^## //')
                    ;;
                "- ["*)
                    display_name=$(echo "$line" | sed -n 's/- \[\([^]]*\)\].*/\1/p')
                    # 考虑到可能带路径，只提取纯文件名
                    filename_with_ext=$(echo "$line" | sed -n 's/.*(\([^)]*\)).*/\1/p')
                    filename=$(basename "$filename_with_ext" .md)
                    
                    if [ -n "$current_section" ] && [ -n "$filename" ] && [ "$filename" != "README" ]; then
                        entries="${entries}${current_section}:${display_name}:${filename}"$'\n'
                    fi
                    ;;
            esac
        done < "$NAV_README"
    else
        echo "[WARNING] Navigation README.md not found in doc/ subdirectories!"
    fi
    echo "$entries"
}

TREE_ENTRIES=$(build_tree_entries)

# 7. 为每个 HTML 注入树形侧边栏
echo "[build-pages-gh] Injecting tree sidebar into HTML files..."
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

    # --- 7a. 构建树状分类侧边栏 ---
    build_sidebar="
<input type=\"checkbox\" id=\"sidebar-toggler\" class=\"sidebar-toggler-input\">
<nav class=\"sidebar\" id=\"sidebar\">
<div class=\"sidebar-header\"><a href=\"${home_href}\">🏠 visionAlgorithm</a></div>
<div class=\"sidebar-tree\">"

    # 按分类分组构建树节点
    prev_section=""
    section_items=""
    
    while IFS= read -r entry; do
        [ -z "$entry" ] && continue
        section=$(echo "$entry" | cut -d: -f1)
        doc_name=$(echo "$entry" | cut -d: -f2)
        doc_file=$(echo "$entry" | cut -d: -f3)

        if [ "$section" != "$prev_section" ] && [ -n "$prev_section" ]; then
            build_sidebar="${build_sidebar}
<details class=\"tree-section\" open>
<summary>📁 ${prev_section}</summary>
<ul>"
            while IFS= read -r item; do
                [ -z "$item" ] && continue
                item_name=$(echo "$item" | cut -d: -f1)
                item_file=$(echo "$item" | cut -d: -f2)
                if [ "$base_name" = "$item_file" ]; then
                    active_class=" class=\"active\""
                else
                    active_class=""
                fi
                build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}${item_file}.html\"${active_class}>📄 ${item_name}</a></li>"
            done <<< "$section_items"
            build_sidebar="${build_sidebar}
</ul>
</details>"
            section_items=""
        fi

        prev_section="$section"
        if [ -z "$section_items" ]; then
            section_items="${doc_name}:${doc_file}"
        else
            section_items="${section_items}"$'\n'"${doc_name}:${doc_file}"
        fi
    done <<< "$TREE_ENTRIES"

    # 处理最后一组分类
    if [ -n "$prev_section" ] && [ -n "$section_items" ]; then
        build_sidebar="${build_sidebar}
<details class=\"tree-section\" open>
<summary>📁 ${prev_section}</summary>
<ul>"
        while IFS= read -r item; do
            [ -z "$item" ] && continue
            item_name=$(echo "$item" | cut -d: -f1)
            item_file=$(echo "$item" | cut -d: -f2)
            if [ "$base_name" = "$item_file" ]; then
                active_class=" class=\"active\""
            else
                active_class=""
            fi
            build_sidebar="${build_sidebar}
<li><a href=\"${link_prefix}${item_file}.html\"${active_class}>📄 ${item_name}</a></li>"
        done <<< "$section_items"
        build_sidebar="${build_sidebar}
</ul>
</details>"
    fi

    # 关闭侧边栏标签
    build_sidebar="${build_sidebar}
</div>
</nav>
<label for=\"sidebar-toggler\" class=\"sidebar-toggle\">☰</label>
<div class=\"page-content\">
"

    # 注入侧边栏到 HTML
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

    echo "[OK] Injected tree sidebar into: $html_file"
done

echo "[build-pages-gh] Done! Output in: $OUTPUT_DIR"
