# SMOKE-TEST ONLY: stub matplotlib's font lookup (the apt matplotlib cannot load next to pip's NumPy 2).
import sys, types
mpl = types.ModuleType("matplotlib"); fm = types.ModuleType("matplotlib.font_manager")
fm.fontManager = types.SimpleNamespace(ttflist=[]); mpl.font_manager = fm
sys.modules.update({"matplotlib": mpl, "matplotlib.font_manager": fm})
from object_detection import train
train.main(sys.argv[1:])
