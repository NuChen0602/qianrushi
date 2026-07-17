#!/usr/bin/env python3
"""从现有最大词库模板生成“CI302只识别发码、绝不播报”的烧录表。"""
import argparse
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(PROJECT_DIR))

from app.book_catalog import BookCatalog
from app.ci302_commands import WAKE_FRAME, SLEEP_FRAME, SLEEP_ACK_FRAME, build_ci302_commands

NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
ET.register_namespace("x", NS)


def cell(column, row_number, value, style):
    attrs = {"r": f"{column}{row_number}", "s": str(style)}
    node = ET.Element(f"{{{NS}}}c", attrs)
    if value == "" or value is None:
        return node
    if isinstance(value, (int, float)):
        node.set("t", "n")
    else:
        node.set("t", "str")
    ET.SubElement(node, f"{{{NS}}}v").text = str(value)
    return node


def replace_sheet(xml_bytes, rows, header_style=10, data_style=18):
    root = ET.fromstring(xml_bytes)
    sheet_data = root.find(f"{{{NS}}}sheetData")
    for old in list(sheet_data):
        sheet_data.remove(old)
    for row_number, values in enumerate(rows, 1):
        row = ET.SubElement(sheet_data, f"{{{NS}}}row", {"r": str(row_number)})
        style = header_style if row_number == 1 else data_style
        for index, value in enumerate(values):
            row.append(cell(chr(ord("A") + index), row_number, value, style))
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def update_table(xml_bytes, last_row):
    root = ET.fromstring(xml_bytes)
    columns = root.find(f"{{{NS}}}tableColumns")
    count = int(columns.attrib["count"])
    root.set("ref", f"A1:{chr(ord('A') + count - 1)}{last_row}")
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def workbook_rows(catalog):
    headers1 = ["语义标签", "命令词", "功能类型", "播报语句", "播报模式", "发送协议", "接收协议", "置信度阈值"]
    system = [
        [1, "欢迎语", "系统词", "", "无", "AA 55 01 00 FB", "AA 55 01 00 FB", 45],
        [2, "休息语", "休眠事件", "", "无", SLEEP_FRAME, SLEEP_ACK_FRAME, 45],
        [3, "你好小亚", "唤醒词", "", "无", WAKE_FRAME, WAKE_FRAME, 45],
        [4, "增大音量", "系统控制词", "", "无", "AA 55 04 00 FB", "AA 55 04 00 FB", 45],
        [5, "减小音量", "系统控制词", "", "无", "AA 55 05 00 FB", "AA 55 05 00 FB", 45],
        [6, "最大音量", "系统控制词", "", "无", "AA 55 06 00 FB", "AA 55 06 00 FB", 45],
        [7, "中等音量", "系统控制词", "", "无", "AA 55 07 00 FB", "AA 55 07 00 FB", 45],
        [8, "最小音量", "系统控制词", "", "无", "AA 55 08 00 FB", "AA 55 08 00 FB", 45],
        [9, "开播报", "系统控制词", "", "无", "AA 55 09 00 FB", "AA 55 09 00 FB", 45],
        [10, "关播报", "系统控制词", "", "无", "AA 55 0A 00 FB", "AA 55 0A 00 FB", 45],
    ]
    commands = build_ci302_commands(catalog)
    command_rows = []
    for semantic_id, command in enumerate(commands, 101):
        command_rows.append([
            semantic_id, command["phrase"], "命令词", "", "无",
            command["frame"], command["frame"], 45,
        ])

    headers2 = ["视频部分", "触发口令/事件", "收到协议", "程序动作", "导航/视觉/传感器动作", "播报模块", "播报内容摘要"]
    flow = [
        ["会话控制", "你好小亚", WAKE_FRAME, "开启麦克风/API兜底监控", "等待CI302标准码或非标准人声", "HS-S77", "请说"],
        ["会话控制", "CI302自动休眠", SLEEP_FRAME, "关闭麦克风/API监控", "结束本轮会话", "HS-S77", "我去休息啦"],
    ]
    for command in commands:
        if command["kind"] == "find_book":
            book = catalog.get(command["book_id"])
            part = "寻书引导"
            action = f"直接执行 FIND_{book['id']}"
            hardware = f"导航至{book['shelf_id']}并识别ArUco ID{book['id']}"
            summary = f"查找《{book['name']}》并播报位置"
        elif command["kind"] == "sleep":
            part, action, hardware, summary = "会话控制", "触发休眠", "结束麦克风/API监控", "我去休息啦"
        else:
            part = "原视频固定任务"
            action = "直接执行 " + command["id"]
            hardware = command.get("mission", command["kind"])
            summary = "由HS-S77播报代码中的固定或动态文本"
        flow.append([part, command["phrase"], command["frame"], action, hardware, "HS-S77", summary])

    notes = [
        ["处理项", "本版处理", "原因", "影响"],
        ["CI302播报语句", "全部留空", "CI302只识别并输出码", "所有语音统一由HS-S77播报"],
        ["标准命令", "24条全部烧录", "标准语句无需ASR/API", "识别后直接执行，延迟最低"],
        ["图书寻书词", "15本全部烧录", "覆盖book_database.json全部ArUco码", "可直接寻找任意登记图书"],
        ["原视频A1-A9", "保持协议兼容", "避免破坏现有演示流程", "百年孤独仍使用A2"],
        ["新增图书协议", "B0-BD", "补齐其余14本书", "代码与表格共用同一码表"],
        ["API兜底", "仅CI302未命中且VAD检测到人声时调用", "避免无意义API请求", "支持同义词和自由问法"],
        ["休眠", "CI302发休眠码或识别退出对话", "结束会话监控", "休眠语由HS-S77播报"],
    ]
    upload = [
        ["项目", "内容"],
        ["用途", "CI302标准命令优先、API兜底、HS-S77统一播报"],
        ["上传工作表", "命令词预处理"],
        ["制作类型", "固定词条"],
        ["声学模型", "V00942_中文_ASR_通用_Pro2_1.3M"],
        ["晶振", "内部 RC"],
        ["通信串口", "UART1"],
        ["波特率", "115200"],
        ["串口协议", "自定义协议"],
        ["UART1电平", "开，RX PA3，TX PA2"],
        ["SDK选项", "不勾"],
        ["自学习功能", "关"],
        ["握手发送", "A5 FA 00 81 0D 00 4A FB"],
        ["握手接收", "A5 FA 00 82 0D 00 4A FB"],
        ["重要注意", "所有播报语句必须为空；CI302只发码，HS-S77负责全部播报"],
    ]
    return [[headers1, *system, *command_rows], [headers2, *flow], notes, upload]


def generate(template, output):
    catalog = BookCatalog(PROJECT_DIR / "config")
    sheets = workbook_rows(catalog)
    replacements = {}
    with zipfile.ZipFile(template, "r") as source:
        for index, rows in enumerate(sheets, 1):
            sheet_name = f"xl/worksheets/sheet{index}.xml"
            table_name = f"xl/tables/table{index}.xml"
            replacements[sheet_name] = replace_sheet(source.read(sheet_name), rows)
            replacements[table_name] = update_table(source.read(table_name), len(rows))
        output.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as target:
            for info in source.infolist():
                target.writestr(info, replacements.get(info.filename, source.read(info.filename)))
    if output.stat().st_size > template.stat().st_size:
        generated_size = output.stat().st_size
        maximum_size = template.stat().st_size
        output.unlink(missing_ok=True)
        raise RuntimeError(f"生成文件 {generated_size} 字节，超过模板上限 {maximum_size} 字节")
    return len(sheets[0]) - 1, len(build_ci302_commands(catalog))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("template", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    total, commands = generate(args.template, args.output)
    print(f"generated={args.output} rows={total} standard_commands={commands} bytes={args.output.stat().st_size}")


if __name__ == "__main__":
    main()
