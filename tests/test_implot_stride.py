"""Regression for the bundled mc_rtc-imgui and ImPlot PlotLine API mismatch."""
from pathlib import Path
import tempfile
import unittest

from smplh_playback.install_bridge import fix_implot_stride


class ImPlotStrideTests(unittest.TestCase):
    def test_stride_argument_and_idempotence(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)
            (source / 'ext/implot').mkdir(parents=True)
            (source / 'ext/mc_rtc-imgui').mkdir(parents=True)
            header = source / 'ext/implot/implot.h'
            header.write_text('ImPlotLineFlags flags=0, int offset=0, int stride=sizeof(T)')
            plot = source / 'ext/mc_rtc-imgui/Plot.cpp'
            call = ('ImPlot::PlotLine(p.label.c_str(), &p.points[0].x, &p.points[0].y, '
                    'p.points.size(), 0, sizeof(Point));')
            plot.write_text(call + '\n' + call)
            fix_implot_stride(source)
            fixed = plot.read_text()
            self.assertEqual(fixed.count('p.points.size(), 0, 0, sizeof(Point))'), 2)
            fix_implot_stride(source)
            self.assertEqual(plot.read_text(), fixed)
            header.write_text('int offset=0, int stride=sizeof(T)')
            plot.write_text(call)
            fix_implot_stride(source)
            self.assertEqual(plot.read_text(), call)


if __name__ == '__main__':
    unittest.main()
