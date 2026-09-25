#!/usr/bin/env python3
"""Generate a simple realistic diagram for the STM32 adaptive SMC project."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def load_font(size, bold=False):
    candidates = [
        r"C:\Windows\Fonts\arialbd.ttf" if bold else r"C:\Windows\Fonts\arial.ttf",
        r"C:\Windows\Fonts\segoeui.ttf" if not bold else r"C:\Windows\Fonts\segoeuib.ttf",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size=size)
        except Exception:
            continue
    return ImageFont.load_default()


def draw_centered_text(draw, box, text, font, fill):
    left, top, right, bottom = box
    lines = text.split("\n")
    line_spacing = 4
    line_heights = []
    line_widths = []
    for line in lines:
        bbox = draw.textbbox((0, 0), line, font=font)
        line_widths.append(bbox[2] - bbox[0])
        line_heights.append(bbox[3] - bbox[1])
    total_h = sum(line_heights) + line_spacing * (len(lines) - 1)
    y = top + (bottom - top - total_h) / 2
    for line, width, height in zip(lines, line_widths, line_heights):
        x = left + (right - left - width) / 2
        draw.text((x, y), line, font=font, fill=fill)
        y += height + line_spacing


def rounded_box(draw, box, fill, outline, radius=28, width=4):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def arrow(draw, start, end, fill="#2f3a4a", width=4, text=None, font=None):
    draw.line([start, end], fill=fill, width=width)
    import math

    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    head_len = 18
    head_angle = 0.45
    p1 = (end[0] - head_len * math.cos(angle - head_angle), end[1] - head_len * math.sin(angle - head_angle))
    p2 = (end[0] - head_len * math.cos(angle + head_angle), end[1] - head_len * math.sin(angle + head_angle))
    draw.polygon([end, p1, p2], fill=fill)
    if text and font:
        mid = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        bbox = draw.textbbox((0, 0), text, font=font)
        text_w = bbox[2] - bbox[0]
        text_h = bbox[3] - bbox[1]
        draw.rounded_rectangle(
            [mid[0] - text_w / 2 - 10, mid[1] - text_h / 2 - 6, mid[0] + text_w / 2 + 10, mid[1] + text_h / 2 + 6],
            radius=10,
            fill="#fbf8f2",
            outline=None,
        )
        draw.text((mid[0] - text_w / 2, mid[1] - text_h / 2), text, font=font, fill=fill)


def main():
    out_path = Path(__file__).with_name("stm32_python_pic.png")

    img = Image.new("RGB", (1600, 900), "#f6f1e8")
    draw = ImageDraw.Draw(img)

    title_font = load_font(42, bold=True)
    subtitle_font = load_font(22)
    box_title_font = load_font(30, bold=True)
    box_body_font = load_font(20)
    small_font = load_font(18)
    mono_font = load_font(18)

    draw.text((800, 55), "Adaptive SMC project flow", font=title_font, fill="#1f2530", anchor="mm")
    draw.text((800, 105), "Firmware sends telemetry over UART, Python reads and plots it", font=subtitle_font, fill="#444d5c", anchor="mm")

    boxes = {
        "sensor": ((90, 250, 410, 430), "#dce9f7", "#5677a4", "VL53L0X sensor", "Measures distance\nfrom the target"),
        "firmware": ((510, 190, 990, 510), "#e9f4e3", "#5e8f5a", "STM32F446RE firmware", "main.c\nSMC + adaptive update\nUART debug line"),
        "motor": ((1110, 250, 1510, 430), "#f7e2df", "#aa6b63", "Stepper / motor", "Moves the mechanism\naccording to control"),
        "python": ((1030, 600, 1500, 760), "#efe4fb", "#7a5ca8", "Python host", "plot_realtime.py\npyserial + matplotlib"),
        "setpoint": ((90, 600, 410, 760), "#fff0cf", "#ba8f33", "Setpoint", "150 mm target\nerror, s, u, a^") ,
    }

    for box, fill, outline, title, body in boxes.values():
        rounded_box(draw, box, fill, outline)
        left, top, right, bottom = box
        draw_centered_text(draw, (left, top + 18, right, top + 92), title, box_title_font, "#1f2530")
        draw_centered_text(draw, (left + 18, top + 96, right - 18, bottom - 18), body, box_body_font, "#2f3a4a")

    arrow(draw, (410, 330), (510, 300), text="distance", font=small_font)
    arrow(draw, (990, 300), (1110, 330), text="actuation", font=small_font)
    arrow(draw, (840, 510), (1260, 600), text="UART telemetry", font=small_font)
    arrow(draw, (410, 680), (510, 420), text="setpoint", font=small_font)

    draw.text(
        (800, 835),
        "Typical firmware output: e:<int> s:<int> v:<int> a^:<int> u:<int> steps:<int> de:<int>",
        font=mono_font,
        fill="#1f2530",
        anchor="mm",
    )

    img.save(out_path)
    print(out_path)


if __name__ == "__main__":
    main()