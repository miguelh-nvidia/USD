#!/pxrpythonsubst

def _modifySettings(appController):
    import os
    cwd = os.getcwd()
    appController._dataModel.viewSettings.showHUD = False
    appController._dataModel.viewSettings.cullBackfaces = True

# Set the background color and refresh the view.
def _setBackgroundColorAction(appController, action):
    action.setChecked(True)
    appController._changeBgColor(action)
    appController._stageView.updateGL()

# Test with a dark grey background color.
def _testGreyDarkBackground(appController):
    _setBackgroundColorAction(appController, appController._ui.actionGrey_Dark)
    appController._takeShot("shapes.png")

def testUsdviewInputFunction(appController):
    _modifySettings(appController)
    _testGreyDarkBackground(appController)
