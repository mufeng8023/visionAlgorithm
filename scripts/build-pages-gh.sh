#!/bin/bash
# 构建 GitHub Pages 的脚本
# 触发环境: 运行于 gh-pages 分支的 GitHub Actions 中
# 与 GitLab 版本的区别:
#   1. 使用独立的 GitHub Pages 工作流触发
#   2. 生成具有完整树形目录结构的侧边栏(从 doc/README.md 解析分类层次)
#   3. 页面内解析 h2/h3 标题作为"页面目录"子节点, 形成完整文档树
# 依赖: pandoc, sed, awk, grep
# 用法: bash scripts/build-pages-gh.sh

set -e

OUTPUT_DIR="${1:-public}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSS_SRC="$SCRIPT_DIR/style-gh.css"
DOC_SRC="$SCRIPT_DIR/../doc"

echo "[build-pages-gh] Output dir: $OUTPUT_DIR"
echo "[build-pages-gh] CSS source: $CSS_SRC"

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

# 3. 转换根目录 README.md 为首页 index.html
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

# 4. 转换 doc/ 下所有 markdown 文件为 HTML
echo "[build-pages-gh] Converting markdown to HTML..."
if [ -d "$DOC_SRC" ]; then
    for md_file in "$DOC_SRC"/*.md; do
        [ -f "$md_file" ] || continue
        base_name=$(basename "$md_file" .md)
        
        # 如果 doc/README.md 仅仅用于导航, 不需要生成独立页面, 可以取消下面这行的注释：
        # [ "$base_name" = "README" ] && continue

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


# 6. 从 doc/README.md 中提取树状目录结构
echo "[build-pages-gh] Building tree structure from doc/README.md..."
build_tree_entries() {
    current_section=""
    entries=""
    if [ -f "$DOC_SRC/README.md" ]; then
        while IFS= read -r line || [ -n "$line" ]; do
            case "$line" in
                "## "*)
                    current_section=$(echo "$line" | sed 's/^## //')
                    ;;
                "- ["*)
                    display_name=$(echo "$line" | sed -n 's/- \[\([^]]*\)\].*/\1/p')
                    filename=$(echo "$line" | sed -n 's/.*(\([^)]*\)\.md).*/\1/p')
                    if [ -n "$current_section" ] && [ -n "$filename" ]; then
                        entries="${entries}${current_section}:${display_name}:${filename}"$'\n'
                    fi
                    ;;
            esac
        done < "$DOC_SRC/README.md"
    fi
    echo "$entries"
}

TREE_ENTRIES=$(build_tree_entries)

# 7. 从生成的 HTML 文件中提取 h2/h3 的 id 和纯文本, 用作当前页面的目录
#    使用 awk 从 HTML 中提取 <h2 id="xxx">text</h2> 或 <h3 id="xxx">text</h3>
echo "[build-pages-gh] Pre-extracting heading TOC from HTML files..."
TOC_DIR=$(mktemp -d)

for html_file in "$OUTPUT_DIR"/doc/*.html; do
    [ -f "$html_file" ] || continue
    base_name=$(basename "$html_file" .html)

    awk -v base="$base_name" -v toc_dir="$TOC_DIR" '
    {
        # 匹配 h2/h3 标题行
        if (match($0, /<h([23])[^>]*id="([^"]+)"[^>]*>(.*)<\/h[23]>/, arr)) {
            level = arr[1]
            id = arr[2]
            # 提取纯文本(去掉内部标签, 如 <code>, <a> 等)
            text = arr[3]
            gsub(/<[^>]*>/, "", text)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", text)
            gsub(/&/, "\\&", text)
            gsub(/</, "<", text)
            gsub(/>/, ">", text)
            gsub(/"/, "\"", text)
            gsub(/&#39;/, "'\''", text)
            if (id != "" && text != "") {
                print level ":" id ":" text >> (toc_dir "/" base ".toc")
            }
        }
    }
    ' "$html_file" 2>/dev/null || true
done

# 8. 为每个 HTML 注入树形侧边栏
#    侧边栏包含:
#    - 首页链接
#    - 树状分类结构(从 doc/README.md 解析)
#    - 当前页面的 h2/h3 标题目录(页面内导航)
echo "[build-pages-gh] Injecting tree sidebar into HTML files..."
for html_file in "$OUTPUT_DIR"/index.html "$OUTPUT_DIR"/doc/*.html; do
    if [ ! -f "$html_file" ]; then
        echo "[WARNING] File not found: $html_file, skipping"
        continue
    fi

    # 判断是首页还是 doc 页面, 决定链接前缀和首页 href;
    case "$html_file" in
        */index.html)
            link_prefix="doc/"
            home_href="index.html"
            ;;
        */doc/*.html)
            link_prefix=""
            home_href="../index.html"
            ;;
    esac

    base_name=$(basename "$html_file" .html)

    # --- 8a. 构建树状分类侧边栏 ---
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

    # --- 8b. 从预提取的 TOC 文件中读取当前页面的标题目录 ---
    if [ "$base_name" != "index" ] && [ -f "$TOC_DIR/${base_name}.toc" ]; then
        toc_block=""
        while IFS=: read -r toc_level toc_id toc_text_rest; do
            # 获取 toc_id 后的所有剩余文本(标题可能含冒号)
            # 重新读取整行, 去掉前两个字段
            _line_="$toc_level:$toc_id:$toc_text_rest"
            toc_text=$(echo "$_line_" | cut -d: -f3-)

            if [ -z "$toc_text" ]; then
                continue
            fi

            if [ "$toc_level" = "2" ]; then
                toc_padding=""
            else
                toc_padding=" style=\"padding-left: 32px; font-size: 13px;\""
            fi
            toc_block="${toc_block}
<li><a href=\"#${toc_id}\"${toc_padding}>${toc_text}</a></li>"
        done < "$TOC_DIR/${base_name}.toc"

        if [ -n "$toc_block" ]; then
            build_sidebar="${build_sidebar}
<details class=\"tree-section tree-section-toc\" open>
<summary>📑 页面目录</summary>
<ul>${toc_block}
</ul>
</details>"
        fi
    fi

    # 关闭侧边栏
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

# 清理临时目录
rm -rf "$TOC_DIR"

echo "[build-pages-gh] Done! Output in: $OUTPUT_DIR"
