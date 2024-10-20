#include    <OSGImGuiHandler.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct OSGImGuiHandler::ImGuiNewFrameCallback : public osg::Camera::DrawCallback
{
    ImGuiNewFrameCallback(OSGImGuiHandler &handler)
        : handler_(handler)
    {

    }

    void operator()(osg::RenderInfo &renderInfo) const override
    {
        handler_.newFrame(renderInfo);
    }

private:

    OSGImGuiHandler &handler_;
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct OSGImGuiHandler::ImGuiRenderCallback : public osg::Camera::DrawCallback
{
    ImGuiRenderCallback(OSGImGuiHandler &handler)
        : handler_(handler)
    {

    }

    void operator()(osg::RenderInfo &renderInfo) const override
    {
        handler_.render(renderInfo);
    }

private:

    OSGImGuiHandler &handler_;
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
OSGImGuiHandler::OSGImGuiHandler()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();

    (void) io;

    init();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
static int ConvertFromOSGKey(int key)
{
    using KEY = osgGA::GUIEventAdapter::KeySymbol;

    switch (key)
    {
    case KEY::KEY_Tab:
        return ImGuiKey_Tab;
    case KEY::KEY_Left:
        return ImGuiKey_LeftArrow;
    case KEY::KEY_Right:
        return ImGuiKey_RightArrow;
    case KEY::KEY_Up:
        return ImGuiKey_UpArrow;
    case KEY::KEY_Down:
        return ImGuiKey_DownArrow;
    case KEY::KEY_Page_Up:
        return ImGuiKey_PageUp;
    case KEY::KEY_Page_Down:
        return ImGuiKey_PageDown;
    case KEY::KEY_Home:
        return ImGuiKey_Home;
    case KEY::KEY_End:
        return ImGuiKey_End;
    case KEY::KEY_Delete:
        return ImGuiKey_Delete;
    case KEY::KEY_BackSpace:
        return ImGuiKey_Backspace;
    case KEY::KEY_Return:
        return ImGuiKey_Enter;
    case KEY::KEY_Escape:
        return ImGuiKey_Escape;
    case KEY::KEY_F1:
        return ImGuiKey_F1;
    case KEY::KEY_F2:
        return ImGuiKey_F2;
    case KEY::KEY_F3:
        return ImGuiKey_F3;
    case KEY::KEY_F4:
        return ImGuiKey_F4;
    case KEY::KEY_F5:
        return ImGuiKey_F5;
    case KEY::KEY_F6:
        return ImGuiKey_F6;
    case KEY::KEY_F7:
        return ImGuiKey_F7;
    case KEY::KEY_F8:
        return ImGuiKey_F8;
    case KEY::KEY_F9:
        return ImGuiKey_F9;
    case KEY::KEY_F10:
        return ImGuiKey_F10;
    case KEY::KEY_F11:
        return ImGuiKey_F11;
    case KEY::KEY_F12:
        return ImGuiKey_F12;
    case KEY::KEY_Shift_L:
        return ImGuiKey_LeftShift;
    case KEY::KEY_Shift_R:
        return ImGuiKey_RightShift;
    case KEY::KEY_Control_L:
        return ImGuiKey_LeftCtrl;
    case KEY::KEY_Control_R:
        return ImGuiKey_RightCtrl;
    case KEY::KEY_Alt_L:
        return ImGuiKey_LeftAlt;
    case KEY::KEY_Alt_R:
        return ImGuiKey_RightAlt;
    default: // Not found
        return -1;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool OSGImGuiHandler::handle(const osgGA::GUIEventAdapter &ea,
                             osgGA::GUIActionAdapter &aa)
{
    if (!initialized)
    {
        auto view = aa.asView();

        if (view)
        {
            setCameraCallbacks(view->getCamera());
            initialized = true;
        }
    }

    ImGuiIO &io = ImGui::GetIO();
    const bool wantCaptureMouse = io.WantCaptureMouse;
    const bool wantCaptureKeyboard = io.WantCaptureKeyboard;

    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
        case osgGA::GUIEventAdapter::KEYUP:
        {
            const bool isKeyDown = ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN;
            const int key = ea.getKey();
            const int special_key = ConvertFromOSGKey(key);

            if (special_key > 0)
            {
                io.KeysDown[special_key] = isKeyDown;
            }

            return wantCaptureKeyboard;
        }
        case (osgGA::GUIEventAdapter::RELEASE):
        case (osgGA::GUIEventAdapter::PUSH):
        {
            io.MousePos = ImVec2(ea.getX(), io.DisplaySize.y - ea.getY());
            mousePressed[0] = ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
            mousePressed[1] = ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON;
            mousePressed[2] = ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
            return wantCaptureMouse;
        }
        default:
        {
            return false;
        }
    }

    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void OSGImGuiHandler::init()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void OSGImGuiHandler::setCameraCallbacks(osg::Camera *camera)
{
    camera->setPreDrawCallback(new ImGuiNewFrameCallback(*this));
    camera->setPostDrawCallback(new ImGuiRenderCallback(*this));
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void OSGImGuiHandler::newFrame(osg::RenderInfo &renderInfo)
{
    ImGui_ImplOpenGL3_NewFrame();

    ImGuiIO &io = ImGui::GetIO();

    osg::Viewport *viewport = renderInfo.getCurrentCamera()->getViewport();
    io.DisplaySize = ImVec2(viewport->width(), viewport->height());

    double currentTime = renderInfo.getView()->getFrameStamp()->getSimulationTime();
    io.DeltaTime = currentTime - time + 1e-7;
    time = currentTime;

    for (int i = 0; i < 3; ++i)
    {
        io.MouseDown[i] = mousePressed[i];
    }

    io.MouseWheel = mouseWheel;
    mouseWheel = 0.0f;

    ImGui::NewFrame();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void OSGImGuiHandler::render(osg::RenderInfo &renderInfo)
{
    drawUI();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
