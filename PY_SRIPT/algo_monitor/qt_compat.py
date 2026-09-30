from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

Signal = QtCore.Signal if hasattr(QtCore, "Signal") else QtCore.pyqtSignal
Slot = QtCore.Slot if hasattr(QtCore, "Slot") else QtCore.pyqtSlot
