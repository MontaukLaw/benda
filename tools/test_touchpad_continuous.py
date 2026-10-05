"""连续权重不受状态硬归零影响，卸载立即停止且重新加载必须锚定。"""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from touchpad import TouchpadTracker
from centroid_filter import signal_centroid, centroid_mm
from tools.test_touchpad_host_signal import HostSource, spot


class ContinuousPositionTests(unittest.TestCase):
    def ready(self, hz=100):
        tracker=TouchpadTracker(threshold=100)
        tracker.set_raw_gains([2.]*20)
        tracker.set_source('host_continuous'); tracker.set_enabled(True)
        source=HostSource(hz)
        for _ in range(hz):
            tracker.feed(source.sample([0]*20,host_input_val=[10000]*20))
        self.assertEqual(tracker.position_reference, (10000.,)*20)
        return tracker,source

    def feed(self, tracker, source, values=None, states=None):
        values=spot(amplitude=5000) if values is None else values
        states=[1 if v>0 else 0 for v in values] if states is None else states
        return tracker.feed(source.sample([v if s==1 else 0 for v,s in zip(values,states)],
                            host_input_val=[10000+v for v in values],host_state=states))

    def test_reference_requires_stable_all_idle_and_freezes_in_recovery(self):
        tracker,source=self.ready()
        for _ in range(100): self.feed(tracker,source,[0]*20,[3]*20)
        self.assertEqual(tracker.position_reference, (10000.,)*20)
        for i in range(100): self.feed(tracker,source,[200*(i%2)]*20,[0]*20)
        self.assertEqual(tracker.position_reference, (10000.,)*20)
        for _ in range(100): self.feed(tracker,source,[100]*20,[0]*20)
        self.assertEqual(tracker.position_reference, (10100.,)*20)

    def test_channel_exit_does_not_zero_position_weight_or_move_centroid(self):
        tracker,source=self.ready()
        tracker.position_decay_enabled=False
        values=[0.]*20; values[0]=values[1]=values[4]=5000.
        for _ in range(10): self.feed(tracker,source,values)
        old=tracker.raw_position; reference=tracker.position_reference
        states=[0]*20; states[0]=states[1]=1; states[4]=2
        for _ in range(10):
            self.assertEqual(self.feed(tracker,source,values,states),(0.,0.))
            self.assertEqual(tracker.raw_position,old)
            self.assertEqual(tracker.position_weights[4],5000.)
        self.assertEqual(old,signal_centroid(values,local_2x2_enabled=False))
        self.assertIs(tracker.position_reference,reference)

    def test_spatial_transfer_is_not_unloading_and_gain_not_applied_twice(self):
        for hz in (20,70,100,200):
            tracker,source=self.ready(hz)
            for _ in range(hz): self.feed(tracker,source,spot(2.,1.5,5000))
            net=0
            for i in range(hz):
                net+=self.feed(tracker,source,spot(2.+.4*(i+1)/hz,1.5,5000))[0]
                self.assertFalse(tracker.release_blocked)
            for _ in range(hz): net+=self.feed(tracker,source,spot(2.4,1.5,5000))[0]
            self.assertAlmostEqual(net,.4*tracker.sensitivity,delta=1.)
            self.assertAlmostEqual(sum(tracker.position_weights),5000.,delta=.01)

    def test_coherent_unload_latches_stops_tail_then_zero_delta_anchor(self):
        tracker,source=self.ready()
        for _ in range(20): self.feed(tracker,source)
        self.assertTrue(tracker.contact)
        states=[1 if v>0 else 0 for v in spot(amplitude=5000)]
        for _ in range(20): self.feed(tracker,source,spot(amplitude=2000),states)
        self.assertTrue(tracker.release_blocked)
        old=tracker.position
        for _ in range(20):
            self.assertEqual(self.feed(tracker,source,spot(2.1,1.5,1900),states),(0,0))
            self.assertEqual(tracker.position,old)
            self.assertTrue(all(axis.filtered is None for axis in tracker.axes))
        anchors=[]
        for _ in range(20):
            was=tracker.contact; delta=self.feed(tracker,source,spot(2.3,1.5,6000))
            if tracker.contact and not was: anchors.append(delta)
        self.assertEqual(anchors,[(0.,0.)])

    def test_adjacency_gate_holds_position_without_calculating_centroid(self):
        tracker,source=self.ready()
        for _ in range(20): self.feed(tracker,source)
        old=tracker.position
        with patch('touchpad.signal_centroid',side_effect=AssertionError('must not calculate')):
            for channels in ((0,),(0,19),(0,5)):
                states=[1 if i in channels else 0 for i in range(20)]
                self.assertEqual(self.feed(tracker,source,states=states),(0,0))
                self.assertEqual(tracker.position,old)
        self.assertIsNotNone(tracker.position_reference)

    def test_gap_generation_and_bad_input_require_new_release_reference(self):
        for reason in ('gap','generation','bad'):
            tracker,source=self.ready()
            for _ in range(20): self.feed(tracker,source)
            if reason=='gap': source.time+=.3
            elif reason=='generation': source.generation+=1
            if reason=='bad': tracker.feed(source.sample(host_input_val=[math.nan]*20))
            else: self.feed(tracker,source)
            self.assertIsNone(tracker.position_reference)
            self.assertFalse(tracker.contact)
            for _ in range(100): self.feed(tracker,source,[0]*20,[0]*20)
            self.assertIsNotNone(tracker.position_reference)

    def test_mm_geometry_and_disabled_channel_exclusion(self):
        self.assertEqual(centroid_mm((.5,.5)),(0.,0.))
        self.assertEqual(centroid_mm((4.5,3.5)),(32.,24.))
        tracker,source=self.ready()
        values=[0.]*20; values[0]=values[1]=5000.; values[19]=100000.
        states=[0]*20; states[0]=states[1]=1; states[19]=-2
        for _ in range(20): self.feed(tracker,source,values,states)
        self.assertEqual(tracker.position_weights[19],0.)
        self.assertEqual(tracker.raw_position,(.5,1.))

    def test_decay_uses_elapsed_time_and_keeps_release_detection_unattenuated(self):
        for hz in (70,100,200):
            tracker,source=self.ready(hz)
            values=[0.]*20;values[0]=values[1]=values[4]=5000.
            for _ in range(hz):self.feed(tracker,source,values)
            old=tracker.position_factors[4]
            states=[0]*20;states[0]=states[1]=1;states[4]=2
            for _ in range(round(.4*hz)):
                previous=tracker.raw_position
                self.feed(tracker,source,values,states)
                self.assertTrue(tracker.contact)
                self.assertFalse(tracker.release_blocked)
                self.assertEqual(tracker.position_unattenuated_weights[4],5000.)
                self.assertLess(math.dist(previous,tracker.raw_position),.05)
            expected=.4+(old-.4)*math.exp(-.4/.08)
            self.assertAlmostEqual(tracker.position_factors[4],expected,places=12)
            self.assertAlmostEqual(tracker.position_weights[4],5000*expected,places=8)

    def test_recover_and_idle_do_not_restore_unloaded_weight(self):
        tracker,source=self.ready()
        values=[0.]*20;values[0]=values[1]=values[4]=5000.
        for _ in range(100):self.feed(tracker,source,values)
        states=[0]*20;states[0]=states[1]=1
        for state in (2,3,0):
            states[4]=state
            for _ in range(30):
                old=tracker.position_factors[4];self.feed(tracker,source,values,states)
                self.assertLessEqual(tracker.position_factors[4],old)
                self.assertGreaterEqual(tracker.position_factors[4],.4)
        before=tracker.position_factors[4];states[4]=1
        self.feed(tracker,source,values,states)
        expected=1+(before-1)*math.exp(-.01/.02)
        self.assertAlmostEqual(tracker.position_factors[4],expected,places=12)
        self.assertLess(tracker.position_factors[4],1.)

    def test_disabled_and_zero_strength_match_original_continuous_weights(self):
        for enabled,strength in ((False,60),(True,0)):
            tracker,source=self.ready();tracker.position_decay_enabled=enabled;tracker.position_decay_percent=strength
            values=[0.]*20;values[0]=values[1]=values[4]=5000.
            states=[0]*20;states[0]=states[1]=1;states[4]=3
            for _ in range(30):self.feed(tracker,source,values,states)
            self.assertEqual(tracker.position_weights,tracker.position_unattenuated_weights)
            self.assertEqual(tracker.raw_position,signal_centroid(values,local_2x2_enabled=False))

    def test_lost_adjacency_holds_cursor_while_decay_continues_then_reanchors(self):
        tracker,source=self.ready()
        for _ in range(100):self.feed(tracker,source)
        old=tracker.position
        states=[0]*20;states[4]=1;states[8]=2
        for _ in range(20):
            self.assertEqual(self.feed(tracker,source,states=states),(0,0))
            self.assertEqual(tracker.position,old)
        self.assertLess(tracker.position_factors[8],.5)
        first=None
        for _ in range(20):
            was=tracker.contact;delta=self.feed(tracker,source)
            if tracker.contact and not was:first=delta
        self.assertEqual(first,(0.,0.))


if __name__=='__main__': unittest.main()
