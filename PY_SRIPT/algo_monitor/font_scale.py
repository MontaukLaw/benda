"""监控窗口的字号比例，供直接绘制文字的视图使用。"""
DEFAULT_FONT_PX = 13
MIN_FONT_PX = 9
MAX_FONT_PX = 26


def font_scale_ratio(widget):
    current = widget
    while current is not None:
        size = current.property('monitorFontPx')
        if size is not None:
            return size / DEFAULT_FONT_PX
        current = current.parentWidget()
    return 1.0


def scaled_font_size(widget, original):
    return max(1, round(original * font_scale_ratio(widget)))
