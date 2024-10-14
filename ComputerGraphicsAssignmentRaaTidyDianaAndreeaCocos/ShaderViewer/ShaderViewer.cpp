#include <windows.h>
#include <iostream>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osgGA/DriveManipulator>

#include <Shader/ShaderUnit.h>
#include <Shader/ShaderAttacher.h>
#include <Shader/ShaderSwitcher.h>

osg::Group* g_pRoot;

int main(int argc, char* argv[])
{
	osgViewer::Viewer viewer;

	Shader::ShaderUnit::setShaderPath("../../shaders/");
	Shader::ShaderUnit::addShader("perPixel");
	Shader::ShaderUnit::addShader("default");

	g_pRoot = new osg::Group();
	g_pRoot->ref();

	osg::ArgumentParser arguments(&argc, argv);
	osg::Node* pLoadedModel = osgDB::readNodeFiles(arguments);

	if (!pLoadedModel)
	{
		std::cout << "Model Failed to Load" << std::endl;
		return 1;
	}

	g_pRoot->addChild(pLoadedModel);

	osg::BoundingSphere bs = g_pRoot->computeBound();

	float dist = tan(bs.radius());

	osg::GraphicsContext::Traits* pTraits = new osg::GraphicsContext::Traits();
	pTraits->x = 20;
	pTraits->y = 20;
	pTraits->width = 600;
	pTraits->height = 480;
	pTraits->windowDecoration = true;
	pTraits->doubleBuffer = true;
	pTraits->sharedContext = 0;
	pTraits->readDISPLAY();
	pTraits->setUndefinedScreenDetailsToDefaultScreen();

	osg::GraphicsContext* pGraphicsContext = osg::GraphicsContext::createGraphicsContext(pTraits);

	osg::Camera* pCamera = viewer.getCamera();
	pCamera->setGraphicsContext(pGraphicsContext);
	pCamera->setViewport(new osg::Viewport(0, 0, pTraits->width, pTraits->height));
	GLenum buffer = pTraits->doubleBuffer ? GL_BACK : GL_FRONT;
	pCamera->setDrawBuffer(buffer);
	pCamera->setReadBuffer(buffer);
	pCamera->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	viewer.setCamera(pCamera);

	Shader::ShaderUnit::createCameraBasedUniformCallbacks(pCamera);

	// add manipulators
	viewer.addEventHandler(new osgViewer::ThreadingHandler);
	viewer.addEventHandler(new Shader::ShaderSwitcher(g_pRoot));
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);
	viewer.addEventHandler(new osgViewer::LODScaleHandler);
	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
	viewer.setSceneData(g_pRoot);

	// present the 3D window
	viewer.realize();

	// start rendering loop
	return viewer.run();
}
