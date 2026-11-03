"""Original schematic target artwork, not manufacturer photographs or CAD."""
from __future__ import annotations

import io
import tkinter as tk

from PIL import Image, ImageDraw

from ..targets import TargetType
from .vehicle import GVT_SPRITE_PATH


def target_image(target: TargetType) -> Image.Image:
    """Normalized top view; physical scaling is applied only by the road painter."""
    if target.is_custom:
        with Image.open(GVT_SPRITE_PATH) as source:
            image = source.convert("RGBA")
        bounds = image.getchannel("A").getbbox()
        return image.crop(bounds) if bounds else image
    image = Image.new("RGBA", (256, 128))
    draw = ImageDraw.Draw(image)
    dark, glass, light = "#182532", "#557787", "#f7e3a8"
    if target.appearance in ("scooter", "motorcycle"):
        draw.rounded_rectangle((0, 53, 47, 75), radius=7, fill=dark)
        draw.rounded_rectangle((208, 53, 255, 75), radius=7, fill=dark)
        draw.polygon(((37, 45), (155, 32), (228, 48), (228, 80),
                      (155, 96), (37, 83)), fill=target.color, outline="#edf2f5")
        if target.appearance == "scooter":
            draw.rounded_rectangle((166, 14, 218, 114), radius=14,
                                   fill=target.color, outline="#9bacb9", width=3)
            draw.rounded_rectangle((190, 25, 205, 103), radius=6, fill=glass)
        else:
            draw.ellipse((137, 34, 196, 94), fill=target.color, outline="#9bacb9", width=3)
        draw.line((177, 0, 177, 127), fill=dark, width=8)
        draw.rounded_rectangle((62, 37, 133, 91), radius=16, fill=dark)
        draw.ellipse((121, 44, 161, 84), fill="#33485c", outline="#e6edf2", width=2)
        draw.line((224, 53, 224, 75), fill=light, width=5)
        draw.line((31, 51, 31, 77), fill="#df6863", width=4)
    else:
        draw.polygon(((255, 24), (233, 0), (26, 0), (0, 24),
                      (0, 103), (26, 127), (233, 127), (255, 103)),
                     fill=target.color, outline="#91a8b8", width=3)
        if target.appearance == "small_car":
            roof = (38, 17, 169, 110)
        elif target.appearance == "suv":
            roof = (21, 14, 184, 113)
        else:
            roof = (62, 19, 178, 108)
        draw.rounded_rectangle(roof, radius=18, fill=glass, outline="#e2edf4", width=2)
        draw.rounded_rectangle((roof[0] + 22, 24, roof[2] - 25, 103),
                               radius=7, fill=target.color)
        for y in (16, 93):
            draw.rectangle((246, y, 254, y + 18), fill=light)
            draw.rectangle((1, y, 8, y + 18), fill="#df6863")
        if target.appearance == "suv":
            draw.line((54, 31, 145, 31), fill="#8496a7", width=4)
            draw.line((54, 96, 145, 96), fill="#8496a7", width=4)
    return image


def target_thumbnail(target: TargetType, master: tk.Misc) -> tk.PhotoImage:
    resampling = getattr(Image, "Resampling", Image)
    image = target_image(target).resize((48, 24), resampling.LANCZOS)
    encoded = io.BytesIO()
    image.save(encoded, format="PNG")
    return tk.PhotoImage(data=encoded.getvalue(), master=master)
