"""Convert one of the Block 4.5 markdown documents into a formatted .docx.

Deliberately small: handles exactly the constructs used in these documents
(#/##/### headings, paragraphs, bullet and numbered lists, fenced code blocks,
pipe tables, block quotes, horizontal rules, **bold** and `inline code`).

Usage:
    python md2docx.py <input.md> <output.docx> [--lang en|cn]
"""

import re
import sys

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

HEAD_COLOR = RGBColor(0x1F, 0x38, 0x64)
CODE_COLOR = RGBColor(0x22, 0x22, 0x22)
RULE_COLOR = "BFBFBF"

FONTS = {
    "en": {"latin": "Calibri", "mono": "Consolas", "ea": "Microsoft YaHei",
           "size": 10.5, "code": 8.5},
    "cn": {"latin": "Microsoft YaHei", "mono": "Consolas", "ea": "Microsoft YaHei",
           "size": 10.0, "code": 8.5},
}


# --------------------------------------------------------------------------- #
# low level helpers
# --------------------------------------------------------------------------- #
def set_run_font(run, latin, ea, size=None):
    run.font.name = latin
    if size is not None:
        run.font.size = Pt(size)
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    rfonts.set(qn("w:ascii"), latin)
    rfonts.set(qn("w:hAnsi"), latin)
    rfonts.set(qn("w:eastAsia"), ea)
    rfonts.set(qn("w:cs"), latin)


def shade(paragraph, hex_fill):
    ppr = paragraph._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    ppr.append(shd)


def bottom_border(paragraph):
    ppr = paragraph._p.get_or_add_pPr()
    borders = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), "6")
    bottom.set(qn("w:space"), "1")
    bottom.set(qn("w:color"), RULE_COLOR)
    borders.append(bottom)
    ppr.append(borders)


INLINE = re.compile(r"(\*\*.+?\*\*|`[^`]+`|(?<![\w*])\*(?:[^*\n]+?)\*(?![\w*]))")


def add_inline(paragraph, text, cfg, bold_all=False, size=None):
    """Render **bold**, `code` and *italic* spans inside one paragraph."""
    for chunk in INLINE.split(text):
        if not chunk:
            continue
        if chunk.startswith("**") and chunk.endswith("**") and len(chunk) > 4:
            run = paragraph.add_run(chunk[2:-2])
            run.bold = True
            set_run_font(run, cfg["latin"], cfg["ea"], size)
        elif chunk.startswith("`") and chunk.endswith("`") and len(chunk) > 2:
            run = paragraph.add_run(chunk[1:-1])
            set_run_font(run, cfg["mono"], cfg["ea"], (size or cfg["size"]) - 1.0)
            run.font.color.rgb = CODE_COLOR
        elif (chunk.startswith("*") and chunk.endswith("*") and len(chunk) > 2
              and not chunk.startswith("**")):
            run = paragraph.add_run(chunk[1:-1])
            run.italic = True
            set_run_font(run, cfg["latin"], cfg["ea"], size)
        else:
            run = paragraph.add_run(chunk)
            if bold_all:
                run.bold = True
            set_run_font(run, cfg["latin"], cfg["ea"], size)


# --------------------------------------------------------------------------- #
# block level rendering
# --------------------------------------------------------------------------- #
def add_table(doc, rows, cfg):
    if not rows:
        return
    # A table whose header row is entirely empty (the metadata block) has no
    # real header - drop that row instead of rendering a blank shaded band.
    has_header = any(c.strip() for c in rows[0])
    body = rows if has_header else rows[1:]
    if not body:
        return

    ncols = max(len(r) for r in body)
    table = doc.add_table(rows=0, cols=ncols)
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.CENTER

    for i, row in enumerate(body):
        table_row = table.add_row()
        cells = table_row.cells
        # keep a row from being split across a page break mid-sentence
        trpr = table_row._tr.get_or_add_trPr()
        cant_split = OxmlElement("w:cantSplit")
        trpr.append(cant_split)
        is_header = has_header and i == 0
        for j in range(ncols):
            text = row[j] if j < len(row) else ""
            cell = cells[j]
            # a cell may contain explicit <br> line breaks
            segments = re.split(r"<br\s*/?>", text)
            for k, segment in enumerate(segments):
                paragraph = cell.paragraphs[0] if k == 0 else cell.add_paragraph()
                paragraph.paragraph_format.space_before = Pt(1)
                paragraph.paragraph_format.space_after = Pt(1)
                add_inline(paragraph, segment, cfg, bold_all=is_header,
                           size=cfg["size"] - 1.0)
                if is_header:
                    shade(paragraph, "E8EDF5")
    doc.add_paragraph()


def add_code_block(doc, lines, cfg):
    for idx, line in enumerate(lines):
        paragraph = doc.add_paragraph()
        pf = paragraph.paragraph_format
        pf.space_before = Pt(0)
        pf.space_after = Pt(0)
        pf.left_indent = Cm(0.4)
        pf.line_spacing = 1.0
        shade(paragraph, "F4F4F4")
        run = paragraph.add_run(line if line else " ")
        set_run_font(run, cfg["mono"], cfg["ea"], cfg["code"])
    doc.add_paragraph()


def add_heading(doc, text, level, cfg):
    # Markdown headings may contain **bold** / `code` markers; headings are
    # already styled, so strip the markers instead of rendering them literally.
    clean = text.replace("**", "").replace("`", "")
    paragraph = doc.add_heading(level=level)
    sizes = {0: 20, 1: 16, 2: 13, 3: 11.5}
    run = paragraph.add_run(clean)
    run.bold = True
    set_run_font(run, cfg["latin"], cfg["ea"], sizes.get(level, 11.5))
    run.font.color.rgb = HEAD_COLOR
    return paragraph


def convert(src, dst, lang="en"):
    cfg = FONTS[lang]
    text = open(src, encoding="utf-8").read()

    doc = Document()
    section = doc.sections[0]
    section.page_height = Cm(29.7)
    section.page_width = Cm(21.0)
    for attr in ("top_margin", "bottom_margin"):
        setattr(section, attr, Cm(2.0))
    for attr in ("left_margin", "right_margin"):
        setattr(section, attr, Cm(2.0))

    normal = doc.styles["Normal"]
    normal.font.name = cfg["latin"]
    normal.font.size = Pt(cfg["size"])
    normal.element.rPr.rFonts.set(qn("w:eastAsia"), cfg["ea"])

    lines = text.splitlines()
    i = 0
    pending_blank = False

    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # fenced code
        if stripped.startswith("```"):
            i += 1
            block = []
            while i < len(lines) and not lines[i].strip().startswith("```"):
                block.append(lines[i])
                i += 1
            i += 1
            add_code_block(doc, block, cfg)
            continue

        # pipe table
        if stripped.startswith("|") and i + 1 < len(lines) and \
                re.match(r"^\|[\s:\-|]+\|$", lines[i + 1].strip()):
            rows = []
            header = [c.strip() for c in stripped.strip("|").split("|")]
            rows.append(header)
            i += 2
            while i < len(lines) and lines[i].strip().startswith("|"):
                rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
                i += 1
            add_table(doc, rows, cfg)
            continue

        # horizontal rule
        if re.match(r"^-{3,}$", stripped) or re.match(r"^\*{3,}$", stripped):
            paragraph = doc.add_paragraph()
            paragraph.paragraph_format.space_before = Pt(2)
            paragraph.paragraph_format.space_after = Pt(6)
            bottom_border(paragraph)
            i += 1
            continue

        # headings
        if stripped.startswith("#"):
            level = len(stripped) - len(stripped.lstrip("#"))
            add_heading(doc, stripped[level:].strip(), level - 1, cfg)
            i += 1
            continue

        # block quote
        if stripped.startswith(">"):
            quote = []
            while i < len(lines) and lines[i].strip().startswith(">"):
                quote.append(lines[i].strip().lstrip(">").strip())
                i += 1
            paragraph = doc.add_paragraph()
            pf = paragraph.paragraph_format
            pf.left_indent = Cm(0.8)
            pf.space_before = Pt(4)
            pf.space_after = Pt(4)
            add_inline(paragraph, " ".join(q for q in quote if q), cfg)
            for run in paragraph.runs:
                run.font.italic = True
            continue

        # bullet list
        if re.match(r"^[-*]\s+", stripped):
            paragraph = doc.add_paragraph(style="List Bullet")
            paragraph.paragraph_format.space_after = Pt(2)
            add_inline(paragraph, re.sub(r"^[-*]\s+", "", stripped), cfg)
            i += 1
            continue

        # numbered list - rendered with an explicit counter, because the
        # built-in "List Number" style shares one numbering definition across
        # the whole document and would keep counting across sections.
        if re.match(r"^\d+\.\s+", stripped):
            counter = 0
            while i < len(lines) and re.match(r"^\d+\.\s+", lines[i].strip()):
                counter += 1
                item = re.sub(r"^\d+\.\s+", "", lines[i].strip())
                paragraph = doc.add_paragraph()
                pf = paragraph.paragraph_format
                pf.left_indent = Cm(0.95)
                pf.first_line_indent = Cm(-0.55)
                pf.space_after = Pt(2)
                number = paragraph.add_run("%d. " % counter)
                set_run_font(number, cfg["latin"], cfg["ea"], cfg["size"])
                add_inline(paragraph, item, cfg)
                i += 1
            continue

        if not stripped:
            pending_blank = True
            i += 1
            continue

        paragraph = doc.add_paragraph()
        paragraph.paragraph_format.space_after = Pt(4)
        add_inline(paragraph, stripped, cfg)
        pending_blank = False
        i += 1

    doc.save(dst)
    return doc


if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]
    lang = "en"
    if "--lang" in sys.argv:
        lang = sys.argv[sys.argv.index("--lang") + 1]
    convert(src, dst, lang)

    check = Document(dst)
    print("wrote %s" % dst)
    print("  paragraphs: %d  tables: %d" % (len(check.paragraphs), len(check.tables)))
