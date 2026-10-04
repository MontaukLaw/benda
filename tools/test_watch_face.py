"""蜂窝表盘的周期边界、惯性、吸附与手势接管行为。"""
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'PY_SRIPT/algo_monitor'))
from watch_face import WatchFaceModel, normalize_settings, DEFAULTS


class WatchFaceTests(unittest.TestCase):
    def fling(self,model):
        model.begin(1)
        for i in range(1,11):model.drag(8,4,1+i*.01)
        model.release(1.11)

    def test_exactly_100_identities_two_sizes_and_periodic_layout(self):
        model=WatchFaceModel()
        icons=model.icons()
        self.assertEqual(len(icons),100)
        self.assertEqual({i['id'] for i in icons},set(range(100)))
        self.assertEqual({i['size'] for i in icons},{84,38})
        self.assertEqual([i['id'] for i in icons if i['selected']],[0])
        self.assertEqual(model.nearest()['id'],0)
        width,height=model.period
        model.begin(0);model.drag(width*10003,-height*8001,.1)
        for old,new in zip(icons,model.icons()):
            self.assertAlmostEqual(old['x'],new['x'],places=6)
            self.assertAlmostEqual(old['y'],new['y'],places=6)

    def test_every_app_can_be_centered_across_both_wrap_boundaries(self):
        model=WatchFaceModel(dict(snap_duration_ms=500))
        now=0
        for app_id in range(100):
            model.snap(now,app_id);model.advance(now+.501)
            self.assertEqual(model.nearest()['id'],app_id)
            self.assertLess(model.nearest()['distance'],1e-7)
            self.assertEqual(model.phase,'idle')
            now+=1

    def test_snap_duration_and_monotonic_easing(self):
        model=WatchFaceModel(dict(inertia_enabled=False,snap_duration_ms=500))
        model.begin(1);model.drag(25,12,1.1);model.release(1.11)
        distances=[]
        for i in range(51):
            model.advance(1.11+i*.01)
            distances.append(model.nearest()['distance'])
        self.assertTrue(all(b<=a+1e-8 for a,b in zip(distances,distances[1:])))
        self.assertLess(distances[-1],1e-7)
        model=WatchFaceModel(dict(inertia_enabled=False,snap_duration_ms=0))
        model.begin(0);model.drag(20,12,.1);model.release(.2)
        self.assertEqual(model.phase,'idle')
        self.assertLess(model.nearest()['distance'],1e-7)

    def test_inertia_distance_speed_duration_caps_then_snap(self):
        model=WatchFaceModel(dict(inertia_max_distance=40,inertia_max_speed=300,
                                 inertia_max_duration_ms=200,inertia_strength=3))
        self.fling(model)
        self.assertEqual(model.phase,'inertia')
        self.assertLessEqual(math.hypot(*model._velocity),300+1e-8)
        self.assertLessEqual(model._duration,.2)
        origin=tuple(model.offset)
        model.advance(model._start+model._duration)
        self.assertLessEqual(math.dist(origin,model.offset),40+1e-6)
        self.assertEqual(model.phase,'snap')
        model.advance(3)
        self.assertEqual(model.phase,'idle')
        self.assertLess(model.nearest()['distance'],1e-7)

    def test_inertia_is_frame_rate_independent(self):
        positions=[]
        for fps in (30,60,120):
            model=WatchFaceModel();self.fling(model)
            for i in range(1,2*fps+1):model.advance(1.11+i/fps)
            positions.append((model.selected,tuple(model.offset)))
        self.assertEqual(positions[0][0],positions[1][0])
        for _,position in positions[1:]:self.assertLess(math.dist(positions[0][1],position),1e-7)

    def test_touch_interrupts_inertia_and_snap_without_jump(self):
        for delay in (.1,.7):
            model=WatchFaceModel();self.fling(model)
            now=1.11+delay;model.advance(now)
            before=tuple(model.offset)
            model.begin(now-.005)
            self.assertEqual(tuple(model.offset),before)
            self.assertEqual(model.phase,'drag')
            model.drag(2,-3,now+.01)
            self.assertLess(math.dist((before[0]+2,before[1]-3),model.offset),1e-7)

    def test_stop_then_lift_does_not_fling_and_tap_selects(self):
        model=WatchFaceModel();model.begin(1);model.drag(20,0,1.01)
        model.drag(0,0,1.3);model.release(1.31)
        self.assertNotEqual(model.phase,'inertia')
        model.begin(2);model.release(2.1,target=17);model.advance(3)
        self.assertEqual(model.nearest()['id'],17)
        model.begin(4);model.drag(30,0,4.1);model.cancel()
        before=tuple(model.offset);model.advance(100)
        self.assertEqual(tuple(model.offset),before)

    def test_config_validation_and_size_constraints(self):
        settings=normalize_settings(dict(spacing=76,large_size=110,medium_size=100,small_size=80,
                                         inertia_strength=float('nan'),snap_duration_ms=-100,
                                         sensor_enabled='yes',outer_zone_percent=40,inner_zone_percent=55))
        self.assertLessEqual(settings['large_size'],76*.94)
        self.assertLessEqual(settings['small_size'],settings['large_size']-4)
        self.assertNotIn('medium_size',settings)
        self.assertNotIn('inner_zone_percent',settings)
        self.assertNotIn('outer_zone_percent',settings)
        self.assertEqual(settings['inertia_strength'],DEFAULTS['inertia_strength'])
        self.assertEqual(settings['snap_duration_ms'],0)
        self.assertIs(settings['sensor_enabled'],True)


if __name__=='__main__':unittest.main()
