#!/pxrpythonsubst

def _modifySettings(appController):
    appController._dataModel.viewSettings.showHUD = False
    appController._dataModel.viewSettings.cullBackfaces = True

def _testShapes(appController):
    appController._takeShot("test.png")

def testUsdviewInputFunction(appController):
    _modifySettings(appController)
    _testShapes(appController)
