#!/usr/bin/env python3
"""Build two offline readers from the project's beginner Markdown documents.

Uses only the Python standard library. This intentionally supports the small
Markdown subset used here: headings, paragraphs, tables, lists, fences, inline
code, bold, links, and the local SVG overview. No simulation is launched.
"""
from html import escape
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
PAGES = {"beginner_guide": "零基础说明书", "code_map": "代码地图"}
INLINE = re.compile(r"`([^`]+)`|!\[([^\]]*)\]\(([^)]+)\)|\[([^\]]+)\]\(([^)]+)\)|\*\*(.+?)\*\*")


def inline(text):
    result, end = [], 0
    for match in INLINE.finditer(text):
        result.append(escape(text[end:match.start()]))
        code, alt, src, label, href, bold = match.groups()
        if code is not None:
            result.append(f"<code>{escape(code)}</code>")
        elif alt is not None:
            if src == "assets/software_overview.svg":
                result.append((DOCS / src).read_text(encoding="utf-8"))
            else:
                result.append(f'<img src="{escape(src, quote=True)}" alt="{escape(alt, quote=True)}">')
        elif label is not None:
            if href in {f"{name}.md" for name in PAGES}:
                href = href[:-3] + ".html"
            result.append(f'<a href="{escape(href, quote=True)}">{escape(label)}</a>')
        else:
            result.append(f"<strong>{inline(bold)}</strong>")
        end = match.end()
    result.append(escape(text[end:]))
    return "".join(result)


def render_markdown(source):
    lines = source.splitlines()
    result, toc = [], []
    i, section, subheading = 0, 0, 0
    section_open = False
    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
            continue
        if line.startswith("```"):
            language, content = line[3:].strip(), []
            i += 1
            while i < len(lines) and not lines[i].startswith("```"):
                content.append(lines[i])
                i += 1
            if i == len(lines):
                raise ValueError("unclosed Markdown code fence")
            title = "PowerShell · 复制完整命令" if language == "powershell" else "路径 / 流程"
            result.append(f'<div class="code-box"><div class="code-label">{title}</div><pre><code>{escape(chr(10).join(content))}</code></pre></div>')
            i += 1
            continue
        heading = re.match(r"^(#{1,3}) (.+)$", line)
        if heading:
            level, title = len(heading[1]), heading[2]
            if level == 2:
                if section_open:
                    result.append("</section>")
                section += 1
                subheading = 0
                anchor = f"section-{section}"
                toc.append((anchor, title))
                result.append(f'<section id="{anchor}">')
                section_open = True
                result.append(f"<h2>{inline(title)}</h2>")
            else:
                subheading += 1
                result.append(f'<h{level} id="heading-{section}-{subheading}">{inline(title)}</h{level}>')
            i += 1
            continue
        if line.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                cells = [s.strip() for s in lines[i].strip().strip("|").split("|")]
                if not all(re.fullmatch(r":?-+:?", cell) for cell in cells):
                    rows.append(cells)
                i += 1
            if any(len(row) != len(rows[0]) for row in rows):
                raise ValueError("inconsistent table columns")
            result.append('<div class="table-scroll"><table><thead><tr>')
            result.extend(f'<th scope="col">{inline(cell)}</th>' for cell in rows[0])
            result.append("</tr></thead><tbody>")
            for row in rows[1:]:
                result.append("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in row) + "</tr>")
            result.append("</tbody></table></div>")
            continue
        ordered = re.match(r"^\d+\. (.*)", line)
        if ordered or line.startswith("- "):
            tag = "ol" if ordered else "ul"
            pattern = r"^\d+\. (.*)" if ordered else r"^- (.*)"
            result.append(f"<{tag}>")
            while i < len(lines) and (item := re.match(pattern, lines[i])):
                result.append(f"<li>{inline(item[1])}</li>")
                i += 1
            result.append(f"</{tag}>")
            continue
        paragraph = []
        while i < len(lines) and lines[i].strip():
            if paragraph and re.match(r"^(#{1,3} |```|\||- |\d+\. )", lines[i]):
                break
            paragraph.append(lines[i])
            i += 1
        text = " ".join(paragraph)
        if text.startswith("!["):
            result.append('<figure class="overview">' + inline(text) + "</figure>")
        else:
            result.append("<p>" + inline(text) + "</p>")
    if section_open:
        result.append("</section>")
    return "\n".join(result), toc


CSS = """
:root{color-scheme:light;--ink:#233846;--muted:#536977;--accent:#146452;--line:#dce5e9;--paper:#fff;--bg:#f3f6f8}
*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:100px}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.85 'Microsoft YaHei','Noto Sans CJK SC',system-ui,sans-serif}
a{color:#146452;text-underline-offset:3px}a:hover{color:#083e32}a:focus-visible,button:focus-visible,input:focus-visible{outline:3px solid #c98013;outline-offset:4px}
.topbar{position:sticky;top:0;z-index:5;background:#193849;color:#fff;border-bottom:1px solid #304f60;padding:13px 32px;display:flex;align-items:center;justify-content:space-between;gap:16px}
.brand{font-size:17px;font-weight:700}.topnav{display:flex;flex-wrap:wrap;gap:10px;align-items:center}.topnav a,.topnav button{font:inherit;font-size:14px;color:#fff;text-decoration:none;padding:5px 13px;border:1px solid #55717f;border-radius:7px;background:transparent;cursor:pointer}.topnav a[aria-current=page]{background:#dceee9;color:#174f43;border-color:#dceee9}
.layout{max-width:1500px;margin:auto;display:grid;grid-template-columns:270px minmax(0,1fr);gap:30px;padding:30px}
aside{align-self:start;position:sticky;top:99px;max-height:calc(100vh - 125px);overflow:auto;padding:8px 12px 16px 0;font-size:14px}aside h2{margin:0 0 12px;font-size:16px;border:0;padding:0}.toc{list-style:none;padding:0;margin:0}.toc a{display:block;padding:7px 11px;border-radius:6px;color:#48606e;text-decoration:none;line-height:1.65}.toc a:hover,.toc a.active{background:#dceee9;color:#174f43}.aside-note{color:var(--muted);border-top:1px solid var(--line);padding-top:15px;margin-top:18px;font-size:13px}
main{min-width:0;background:var(--paper);border:1px solid var(--line);border-radius:14px;padding:38px 46px 48px;box-shadow:0 5px 24px #19384905}.eyebrow{color:var(--accent);font-size:13px;font-weight:700;letter-spacing:.1em}h1{font-size:32px;line-height:1.5;margin:12px 0 22px;letter-spacing:-.02em}h2{font-size:25px;line-height:1.5;margin:0 0 22px;padding-top:8px}h3{font-size:19px;line-height:1.65;margin:28px 0 12px;color:#195446}p{margin:14px 0}strong{font-weight:700;color:#163a47}section{border-top:1px solid var(--line);padding-top:30px;margin-top:36px;scroll-margin-top:95px}li{padding-left:4px;margin-bottom:12px}ol,ul{padding-left:25px}
code{font-family:Consolas,'Microsoft YaHei',monospace;font-size:.91em;background:#edf3f5;border-radius:4px;padding:2px 5px;overflow-wrap:anywhere}pre{margin:0;padding:16px 20px;white-space:pre-wrap;overflow-wrap:anywhere;line-height:1.75}pre code{padding:0;background:none;font-size:14px;color:#e9f3f7}.code-box{background:#203a49;border-radius:9px;margin:18px 0;overflow:hidden}.code-label{font-size:12px;color:#c7dde8;padding:7px 20px;border-bottom:1px solid #486371;background:#2a4656}
.table-scroll{overflow:auto;margin:20px 0;border:1px solid var(--line);border-radius:8px}table{border-collapse:collapse;width:100%;font-size:14px;line-height:1.8;table-layout:auto}th,td{padding:12px 14px;text-align:left;vertical-align:top;overflow-wrap:anywhere;border-bottom:1px solid var(--line)}th{background:#edf4f6;font-weight:700;color:#264d60}td:first-child{min-width:140px;width:30%;color:#243f4e}tr:last-child td{border-bottom:0}tbody tr:nth-child(even){background:#fbfcfd}td code{font-size:13px}figure{margin:24px 0}figure svg,figure img{width:100%;height:auto;display:block}.page-footer{margin-top:35px;padding-top:22px;border-top:1px solid var(--line);font-size:13px;color:var(--muted)}.mobile-toc{display:none}
@media(min-width:1400px){main{padding-left:60px;padding-right:60px}}
@media(max-width:1050px){.layout{grid-template-columns:220px minmax(0,1fr);gap:20px;padding:22px}main{padding:28px}h1{font-size:28px}.topbar{padding:12px 22px}}
@media(max-width:780px){.topbar{position:static;align-items:flex-start;flex-direction:column;padding:16px}.layout{display:block;padding:16px}aside{position:static;max-height:none;padding:0 0 20px}.toc{display:grid;grid-template-columns:1fr 1fr;gap:2px}.aside-note{display:none}main{padding:25px 20px}h1{font-size:25px}h2{font-size:22px}table{min-width:530px}td:first-child{min-width:130px}figure{overflow:auto}figure svg{min-width:690px}section{scroll-margin-top:20px}html{scroll-padding-top:20px}.topnav a,.topnav button{font-size:13px}}
@media print{@page{size:A4;margin:16mm}body{background:#fff;font-size:10pt;line-height:1.65}.topbar,aside,.page-footer{display:none}.layout{display:block;padding:0;max-width:none}main{border:0;box-shadow:none;padding:0}.eyebrow{font-size:9pt}h1{font-size:23pt}h2{font-size:16pt}h3{font-size:12pt}h1,h2,h3{break-after:avoid}section{padding-top:18px;margin-top:22px}table{font-size:9pt;min-width:0!important}th,td{padding:7px 9px}thead{display:table-header-group}tr,figure,.code-box{break-inside:avoid}.table-scroll{overflow:visible;border-radius:0}figure svg{min-width:0!important}a{color:inherit;text-decoration:none}.code-box{background:#f1f5f7;border:1px solid #ccc}.code-label{background:#e8eef1;color:#333}pre code{color:#222}pre{padding:10px;line-height:1.5}pre code{font-size:9pt}p{orphans:3;widows:3}}
"""


def build():
    for slug, label in PAGES.items():
        body, toc = render_markdown((DOCS / f"{slug}.md").read_text(encoding="utf-8"))
        navigation = "".join(f'<a href="{name}.html"' + (' aria-current="page"' if name == slug else '') + f'>{title}</a>' for name, title in PAGES.items())
        contents = "".join(f'<li><a href="#{anchor}">{escape(title)}</a></li>' for anchor, title in toc)
        document = f'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="description" content="面向零基础使用者的装载机仿真软件说明与代码地图。"><title>{label} · 装载机仿真软件</title><style>{CSS}</style></head>
<body><header class="topbar"><div class="brand">装载机仿真软件 · 学习与查阅</div><nav class="topnav" aria-label="文档切换">{navigation}<button type="button" onclick="window.print()">打印 / 保存 PDF</button></nav></header>
<div class="layout"><aside aria-label="章节目录"><h2>本页目录</h2><ol class="toc">{contents}</ol><p class="aside-note">初次阅读先看整体和启动方法。<br>用 Ctrl+F 搜索文件名或关键词。<br><br>离线阅读 · 无需启动仿真<br>文档快照：2026-09-06</p></aside>
<main><div class="eyebrow">从使用方法，到代码职责</div>{body}<footer class="page-footer">本页由 {slug}.md 生成。修改 Markdown 后运行 tools/docs/build_beginner_docs.py 更新阅读版。模型与算法仍在开发，具体数值以最新代码和实测报告为准。</footer></main></div>
<script>
const links=[...document.querySelectorAll('.toc a')];
if('IntersectionObserver' in window){{
  const observer=new IntersectionObserver(entries=>{{
    const item=entries.find(e=>e.isIntersecting);
    if(item)links.forEach(a=>a.classList.toggle('active',a.hash==='#'+item.target.id));
  }},{{rootMargin:'-10% 0px -65% 0px'}});
  document.querySelectorAll('main section').forEach(s=>observer.observe(s));
}}
</script></body></html>'''
        (DOCS / f"{slug}.html").write_text(document, encoding="utf-8")
        print(f"Built docs/{slug}.html ({len(toc)} chapters)")


if __name__ == "__main__":
    build()
