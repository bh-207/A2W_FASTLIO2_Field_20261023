"""Export the existing Chinese report and all five plots without altering evidence."""
from pathlib import Path
import re,html,hashlib,json
from PIL import Image as PILImage
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,Image,PageBreak
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from pypdf import PdfReader

ROOT=Path(__file__).resolve().parent
DEST=ROOT/'output/pdf'
DEST.mkdir(parents=True,exist_ok=True)
PDF=DEST/'现场828KF地图质量分析报告.pdf'
SOURCE=ROOT/'REPORT_ZH.txt'
pdfmetrics.registerFont(TTFont('CJK','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'))
pdfmetrics.registerFont(TTFont('Latin','/usr/share/fonts/truetype/freefont/FreeSans.ttf'))
INK=colors.HexColor('#172b42');BLUE=colors.HexColor('#175a89');LIGHT=colors.HexColor('#eaf1f7')
styles={
    'title':ParagraphStyle('Title',fontName='CJK',fontSize=21,leading=29,textColor=INK,spaceAfter=12),
    'sub':ParagraphStyle('Sub',fontName='CJK',fontSize=9,leading=14,textColor=colors.HexColor('#53677a'),spaceAfter=12,wordWrap='CJK'),
    'h':ParagraphStyle('Heading',fontName='CJK',fontSize=13,leading=20,textColor=BLUE,spaceBefore=15,spaceAfter=8,keepWithNext=True,wordWrap='CJK'),
    'body':ParagraphStyle('Body',fontName='CJK',fontSize=10,leading=16.5,textColor=INK,spaceAfter=8,wordWrap='CJK',splitLongWords=True,allowWidows=0,allowOrphans=0),
    'cell':ParagraphStyle('Cell',fontName='CJK',fontSize=8,leading=12,textColor=INK,wordWrap='CJK'),
    'caption':ParagraphStyle('Caption',fontName='CJK',fontSize=9,leading=14,textColor=colors.HexColor('#53677a'),spaceAfter=10,wordWrap='CJK'),
}
def markup(s):
    runs=[];run='';previous=None
    for ch in s:
        font='CJK' if ord(ch) in pdfmetrics.getFont('CJK').face.charToGlyph else 'Latin'
        assert ord(ch) in pdfmetrics.getFont(font).face.charToGlyph,repr(ch)
        if previous is not None and font!=previous:
            runs.append(f'<font name="{previous}">{html.escape(run)}</font>');run=''
        run+=ch;previous=font
    if run:runs.append(f'<font name="{previous}">{html.escape(run)}</font>')
    return ''.join(runs)
def para(s,kind='body'):
    return Paragraph(markup(s),styles[kind])
W,H=A4;M=40;WIDTH=W-2*M
def footer(c,doc):
    c.saveState();c.setStrokeColor(colors.HexColor('#d7e1eb'));c.line(M,36,W-M,36)
    fstyle=ParagraphStyle('Footer',fontName='Latin',fontSize=8,leading=10,textColor=colors.HexColor('#6b7c8c'))
    p=Paragraph(markup('A2W FAST-LIO2 | 固定外参现场地图复核 | 2026-10-23'),fstyle)
    p.wrap(WIDTH-50,12);p.drawOn(c,M,22)
    p=Paragraph(markup(f'第 {doc.page} 页'),fstyle);p.wrap(45,12);p.drawOn(c,W-M-35,22);c.restoreState()
def table(lines):
    data=[]
    for line in lines:
        cells=[v.strip() for v in line.strip().strip('|').split('|')]
        if all(re.fullmatch(r':?-+:?',v) for v in cells):continue
        cells=[v.replace('source→target','源→目标') for v in cells]
        data.append([para(v,'cell') for v in cells])
    n=len(data[0])
    ratios={7:[.115,.125,.105,.105,.105,.11,.235],5:[.17,.12,.235,.235,.24],4:[.17,.17,.26,.40]}[n]
    t=Table(data,colWidths=[WIDTH*v for v in ratios],repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),LIGHT),('VALIGN',(0,0),(-1,-1),'TOP'),
        ('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LINEBELOW',(0,0),(-1,0),.7,colors.HexColor('#afc5d9')),
        ('LINEBELOW',(0,1),(-1,-1),.3,colors.HexColor('#dce5ed')),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f8fafc')]),
    ]));return t

text=SOURCE.read_text()
lines=text.splitlines();story=[para('现场828KF地图质量分析报告','title'),para('固定外参现场会话 fixed_short_20261023_001 · 原报告完整正文及全部五幅对照图','sub')]
story.append(para('导出说明：保留原报告数字、失败证据、不确定性及判断边界；本PDF没有新增建图实验或改变结论。','caption'))
i=1
while i<len(lines):
    line=lines[i].strip()
    if not line:i+=1;continue
    if line.startswith('|'):
        group=[]
        while i<len(lines) and lines[i].strip().startswith('|'):
            group.append(lines[i]);i+=1
        story.extend([table(group),Spacer(1,10)]);continue
    if re.match(r'^\d+\. ',line):story.append(para(line,'h'))
    else:story.append(para(line))
    i+=1

def figure(name,title,caption,maxheight):
    path=ROOT/name
    with PILImage.open(path) as im:w,h=im.size
    scale=min(WIDTH/w,maxheight/h)
    story.extend([para(title,'h'),Image(str(path),width=w*scale,height=h*scale,hAlign='CENTER'),Spacer(1,8),para(caption,'caption')])

story.append(PageBreak())
figure('seam_before_after.png','图1｜往返高度与法向接缝：优化前后',
       '相同扫描点身份、同一全局坐标基准，不执行局部ICP。蓝色为前端raw，橙色为后端optimized；高度柱为中位数，横杠为P95。不同局部平面的坡度保持自由；物理表面身份未经现场测量认证。',235)
figure('whole_map_same_points.png','图2｜整图俯视与原生YZ侧视',
       '按相同body点索引抽取1/20作显示，左右使用相同坐标尺度与颜色。整图高度同时包含地面、设备和屋顶；不以全图Z跨度判断漂移。',265)
story.append(PageBreak())
figure('local_return_sections.png','图3｜八个局部区域的实际往返点云',
       '蓝色为首程训练扫描，绿色为首程留出扫描，橙色为返程。各子图采用相同坐标尺度，竖直轴沿固定初始反重力方向；没有逐区域配准或修平。最后一区排除回环配准子图的±5关键帧邻域。',610)
story.append(PageBreak())
figure('trajectory_and_backend.png','图4｜路线、高度与后端非统一修正',
       '分别显示raw/optimized路线与原生Z、沿同次INIT反重力方向的高度，以及最终位姿图相对raw的修正。路线端点是否为同一物理点未确认，16.10米端点距离不能称为闭环误差。',560)
story.append(PageBreak())
figure('state_diagnostics.png','图5｜重力、姿态、偏置与Z轨迹',
       '采用统一传感器相对时间。g方向变化、ba/bg变化均来自滤波状态估计；roll/pitch包含真实行走、坡道与机体调姿，不是独立姿态误差真值。固定外参在记录精度内未变化。',610)

doc=SimpleDocTemplate(str(PDF),pagesize=A4,rightMargin=M,leftMargin=M,topMargin=38,bottomMargin=50,
    title='现场828KF地图质量分析报告',author='A2W FAST-LIO2 项目',subject='回环、往返高度接缝、重力方向和采集完整性复核')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
reader=PdfReader(str(PDF));extracted='\n'.join(p.extract_text() for p in reader.pages)
assert '\x00' not in extracted
compact=re.sub(r'\s+','',extracted)
for phrase in ['828','6,512,118','10.94','12.66','14.00','7.333','4.613','capture_degraded=true','12条Bag-only','16条callback-only']:
    assert phrase in compact,phrase
checks={'pdf':str(PDF),'page_count':len(reader.pages),'bytes':PDF.stat().st_size,
    'pdf_sha256':hashlib.sha256(PDF.read_bytes()).hexdigest(),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    'text_assertions_passed':True,'figure_count':5,'source_unchanged':SOURCE.read_text()==text}
(DEST/'export_checks.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(checks,ensure_ascii=False,indent=2))
