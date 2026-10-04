"""循环蜂窝表盘的布局和运动模型，不依赖Qt、串口或系统鼠标。"""
import math
from collections import deque


RADIUS = 280.0
DEFAULTS = dict(drag_speed=300, inertia_enabled=True, inertia_strength=.7,
                inertia_decay_ms=220, inertia_max_distance=260, inertia_max_speed=1600,
                inertia_max_duration_ms=800, inertia_stop_speed=25, snap_duration_ms=500,
                spacing=92, large_size=84, small_size=38, size_transition_ms=120,
                center_glow_strength=85, inactive_opacity=50, focus_transition_ms=140,
                tap_threshold=6, reverse_x=False, reverse_y=False,
                show_numbers=True, sensor_enabled=True)
LIMITS = dict(drag_speed=(30,1500), inertia_strength=(0,3), inertia_decay_ms=(40,1500),
              inertia_max_distance=(0,1500), inertia_max_speed=(100,5000),
              inertia_max_duration_ms=(50,3000), inertia_stop_speed=(5,150),
              snap_duration_ms=(0,2000), spacing=(76,140), large_size=(30,110),
              small_size=(16,80), size_transition_ms=(0,500), tap_threshold=(2,30),
              center_glow_strength=(0,100), inactive_opacity=(10,100), focus_transition_ms=(0,500))


def normalize_settings(values):
    settings = dict(DEFAULTS)
    if not isinstance(values, dict):
        return settings
    for key, default in DEFAULTS.items():
        value = values.get(key, default)
        if isinstance(default, bool):
            settings[key] = value if isinstance(value, bool) else default
        else:
            try:
                value = float(value)
                if not math.isfinite(value):
                    continue
                low,high = LIMITS[key]
                value = max(low,min(high,value))
                settings[key] = round(value,2) if isinstance(default,float) else round(value)
            except (TypeError,ValueError,OverflowError):
                pass
    # 沿用已有大小配置，统一为选中与未选中两档，并留出图标间隙。
    settings['large_size'] = min(settings['large_size'], int(settings['spacing']*.94))
    settings['small_size'] = min(settings['small_size'], settings['large_size']-4)
    return settings


class WatchFaceModel:
    """100个固定身份图标组成10×10周期网格，偏移量周期化以支持无限拖动。"""
    def __init__(self, settings=None):
        self.settings = normalize_settings(settings)
        self.offset = [0.0,0.0]
        self.phase = 'idle'
        self.selected = 0
        self._history = deque()
        self._drag_total = [0.0,0.0]
        self.drag_distance = 0.0
        self._last_move = None
        self._start = 0.0
        self._duration = 0.0
        self._origin = (0.0,0.0)
        self._delta = (0.0,0.0)
        self._velocity = (0.0,0.0)
        self._animation_clock = 0.0

    @property
    def period(self):
        pitch = self.settings['spacing']
        return (10*pitch, 10*pitch*math.sqrt(3)/2)

    def _set_offset(self, x, y):
        self.offset = [(v+period/2)%period-period/2 for v,period in zip((x,y),self.period)]

    def icons(self):
        pitch = self.settings['spacing']
        width,height = self.period
        icons=[]
        for index in range(100):
            row,col = divmod(index,10)
            x = (col*pitch+(row%2)*pitch/2+self.offset[0]+width/2)%width-width/2
            y = (row*pitch*math.sqrt(3)/2+self.offset[1]+height/2)%height-height/2
            distance = math.hypot(x,y)
            icons.append(dict(id=index,x=x,y=y,distance=distance))
        # 拖动和惯性时选择最近项；吸附期间锁定点击或吸附的目标。
        focused = self.selected if self.phase == 'snap' else min(icons,key=lambda item:(item['distance'],item['id']))['id']
        for icon in icons:
            icon['selected'] = icon['id'] == focused
            icon['tier'] = 'large' if icon['selected'] else 'small'
            icon['size'] = self.settings[icon['tier']+'_size']
        return icons

    def nearest(self):
        return min(self.icons(),key=lambda item:(item['distance'],item['id']))

    def focused(self):
        return next(icon for icon in self.icons() if icon['selected'])

    def configure(self, values, now):
        self.advance(now)
        focused = self.nearest()['id']
        self.settings = normalize_settings(values)
        self.cancel()
        self._set_offset(*self.offset)
        self.snap(now, focused)

    def cancel(self):
        self.phase = 'idle'
        self._history.clear()
        self._last_move = None

    def begin(self, now):
        self.advance(now)
        self.phase = 'drag'
        self._drag_total = [0.0,0.0]
        self.drag_distance = 0.0
        self._last_move = None
        self._history = deque([(now,0.0,0.0)])

    def drag(self, dx, dy, now):
        if self.phase != 'drag' or not all(math.isfinite(v) for v in (dx,dy,now)):
            return
        self._set_offset(self.offset[0]+dx,self.offset[1]+dy)
        self._drag_total[0] += dx
        self._drag_total[1] += dy
        self.drag_distance += math.hypot(dx,dy)
        if math.hypot(dx,dy) > .01:
            self._last_move = now
        self._history.append((now,*self._drag_total))
        while len(self._history)>2 and now-self._history[1][0]>.08:
            self._history.popleft()

    def release(self, now, target=None):
        if self.phase != 'drag':
            return
        is_tap = self.drag_distance <= self.settings['tap_threshold']
        if is_tap or not self.settings['inertia_enabled'] or self._last_move is None or now-self._last_move>.1:
            self.snap(now, target if is_tap else None)
            return
        start,x,y = self._history[0]
        elapsed = now-start
        if elapsed < .008:
            self.snap(now)
            return
        velocity = tuple((value-old)/elapsed*self.settings['inertia_strength']
                         for value,old in zip(self._drag_total,(x,y)))
        speed = math.hypot(*velocity)
        limit = self.settings['inertia_max_speed']
        if speed>limit:
            velocity = tuple(v*limit/speed for v in velocity)
            speed = limit
        stop = self.settings['inertia_stop_speed']
        distance_limit = self.settings['inertia_max_distance']
        if speed<=stop or distance_limit<=0:
            self.snap(now)
            return
        tau = self.settings['inertia_decay_ms']/1000
        duration = min(self.settings['inertia_max_duration_ms']/1000,tau*math.log(speed/stop))
        if distance_limit < speed*tau:
            duration = min(duration,-tau*math.log1p(-distance_limit/(speed*tau)))
        self.phase = 'inertia'
        self._start,self._duration = now,duration
        self._origin,self._velocity = tuple(self.offset),velocity

    def snap(self, now, target=None):
        icon = self.nearest() if target is None else self.icons()[target%100]
        self.selected = icon['id']
        self._origin = tuple(self.offset)
        self._delta = (-icon['x'],-icon['y'])
        self._start = now
        self._duration = self.settings['snap_duration_ms']/1000
        if icon['distance']<.01 or self._duration<=0:
            self._set_offset(self.offset[0]-icon['x'],self.offset[1]-icon['y'])
            self.phase = 'idle'
        else:
            self.phase = 'snap'

    def advance(self, now):
        # 串口事件可能比绘图定时器稍晚送达，接管动画时禁止回放到旧时刻。
        now=max(now,self._animation_clock)
        self._animation_clock=now
        if self.phase == 'inertia':
            elapsed = max(0,min(self._duration,now-self._start))
            tau = self.settings['inertia_decay_ms']/1000
            amount = tau*(-math.expm1(-elapsed/tau))
            self._set_offset(*(origin+speed*amount for origin,speed in zip(self._origin,self._velocity)))
            if now >= self._start+self._duration:
                self.snap(self._start+self._duration)
        if self.phase == 'snap':
            progress = 1 if now>=self._start+self._duration else max(0,(now-self._start)/self._duration)
            eased = progress**3*(10+progress*(-15+6*progress))
            self._set_offset(*(origin+delta*eased for origin,delta in zip(self._origin,self._delta)))
            if progress>=1:
                self.phase = 'idle'
