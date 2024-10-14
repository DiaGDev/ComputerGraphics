#include <windows.h>
#include <iostream>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osgGA/DriveManipulator>

#include <Common/Facade.h>
#include <Common/Printer.h>
#include <Common/AssetLibrary.h>
#include <Common/FacadeManufactory.h>

#include <TrafficSystem/RoadFacadeFactory.h>
#include <TrafficSystem/RoadFacade.h>
#include <TrafficSystem/TrafficLightFacadeFactory.h>
#include <TrafficSystem/TrafficLightFacade.h>
#include <TrafficSystem/CarFacadeFactory.h>
#include <TrafficSystem/CarFacade.h>

#include <TrafficSystem/AnimationPointFinder.h>
#include <TrafficSystem/Collider.h>

#include <Assignment/ControllableTrafficLightFacade.h>
#include <Assignment/ControllableTrafficLightFacadeFactory.h>
#include <Assignment/RoadTileLightsFacadeFactory.h>
#include <Assignment/RoadTileLightsFacade.h>


#include "InteractionHandler.h"

// the root of the scene tree
osg::Group* g_pRoot;

bool g_bNames = false;
bool g_bAnimationPoints = false;
bool g_bAnimationNames = false;

void keyFunction(char c)
{
	switch (c)
	{
	case 'h':
		std::cout << "Assignment Viewer - key options" << std::endl;
		std::cout << "\tp - print scene tree to console" << std::endl;
		std::cout << "\td - toggle visibility of the collider detection boxes" << std::endl;
		std::cout << "\tn - toggle name display for road tiles" << std::endl;
		std::cout << "\ta - toggle animation point display for road tiles" << std::endl;
		std::cout << "\tz - toggle animation point name display for road tiles" << std::endl;

		break;
	case 'p':
	{
		Common::Printer printer;
		printer.traverse(*g_pRoot);
	}
	break;
	case 'd':
		TrafficSystem::Collider::toggleVisible();
		break;
	case 'n':
		g_bNames = !g_bNames;
		for (Common::FacadeMap::iterator it = Common::Facade::facades().begin(); it != Common::Facade::facades().end(); it++)
			if (TrafficSystem::RoadFacade* pRF = dynamic_cast<TrafficSystem::RoadFacade*>(it->second))
				pRF->enableNames(g_bNames);
		break;
	case 'a':
		g_bAnimationPoints = !g_bAnimationPoints;
		for (Common::FacadeMap::iterator it = Common::Facade::facades().begin(); it != Common::Facade::facades().end(); it++)
			if (TrafficSystem::RoadFacade* pRF = dynamic_cast<TrafficSystem::RoadFacade*>(it->second))
				pRF->enableAnimationPoints(g_bAnimationPoints);
		break;
	case 'z':
		g_bAnimationNames = !g_bAnimationNames;
		for (Common::FacadeMap::iterator it = Common::Facade::facades().begin(); it != Common::Facade::facades().end(); it++)
			if (TrafficSystem::RoadFacade* pRF = dynamic_cast<TrafficSystem::RoadFacade*>(it->second))
				pRF->enableAnimationIDs(g_bAnimationNames);
		break;
	}
}

int main()
{
	osgViewer::Viewer viewer;

	Common::FacadeManufactory::start();
	Common::FacadeManufactory::instance()->addFactory("RoadTile", new Assignment::RoadTileLightsFacadeFactory());
	Common::FacadeManufactory::instance()->addFactory("TrafficLight", new Assignment::ControllableTrafficLightFacadeFactory());
	Common::FacadeManufactory::instance()->addFactory("Car", new TrafficSystem::CarFacadeFactory());
	TrafficSystem::Collider::toggleVisible();
	Common::AssetLibrary::start();

	Common::AssetLibrary::instance()->loadAsset("Road-Straight", "../../Data/roadStraight.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-TJunction", "../../Data/roadTJunction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-XJunction", "../../Data/roadXJUnction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-Curve", "../../Data/roadCurve.osgb");
	Common::AssetLibrary::instance()->loadAsset("TrafficLight", "../../Data/raaTrafficLight.osgb");

	g_pRoot = new osg::Group();
	g_pRoot->ref();

	osg::Matrixf m;

	m.makeTranslate(0.0f, 0.0f, 0.0f);

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadLightsJunction", Common::AssetLibrary::instance()->cloneAsset("Road-XJunction"), m, true)->root());


	if (Assignment::RoadTileLightsFacade* pRTL = dynamic_cast<Assignment::RoadTileLightsFacade*>(Common::Facade::findFacade("RoadLightsJunction")))
	{
		pRTL->enableAnimationIDs(true);
		pRTL->enableNames(true);
		pRTL->enableAnimationPoints(true);

		osg::Matrix m0, m1, m2, m3;

		m0 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(-90.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(-180.0f, 180.0f, 0.0f);
		m1 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(0.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(-180.0f, -180.0f, 0.0f);
		m2 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(180.0f, -180.0f, 0.0f);
		m3 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(180.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(180.0f, 180.0f, 0.0f);

		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "TL0", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), m0, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "TL1", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), m1, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "TL2", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), m2, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "TL3", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), m3, true)));

	}



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
	viewer.setCamera(pCamera);

	// add event handlers
	viewer.addEventHandler(new osgViewer::ThreadingHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);
	viewer.addEventHandler(new osgViewer::LODScaleHandler);
	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
	viewer.addEventHandler(new InteractionHandler(keyFunction));

	// set the scene to render
	viewer.setSceneData(g_pRoot);

	// present the 3D window
	viewer.realize();

	// start rendering loop
	return viewer.run();
}
