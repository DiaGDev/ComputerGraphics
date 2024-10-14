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

#include <Assignment/Example.h>

#include <TrafficSystem/AnimationPointFinder.h>
#include <TrafficSystem/Collider.h>

#include "ExampleInteractionHandler.h"

// the root of the scene tree
osg::Group* g_pRoot;

#define SIMPLE // comment out this line for the assignment main function

#ifndef SIMPLE

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
	Assignment::Example e;

	osgViewer::Viewer viewer;

	Common::FacadeManufactory::start();
	Common::FacadeManufactory::instance()->addFactory("RoadTile", new TrafficSystem::RoadFacadeFactory());
	Common::FacadeManufactory::instance()->addFactory("TrafficLight", new TrafficSystem::TrafficLightFacadeFactory());
	Common::FacadeManufactory::instance()->addFactory("Car", new TrafficSystem::CarFacadeFactory());
	TrafficSystem::Collider::toggleVisible();
	Common::AssetLibrary::start();

	Common::AssetLibrary::instance()->loadAsset("Road-Straight", "../../Data/roadStraight.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-TJunction", "../../Data/roadTJunction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-XJunction", "../../Data/roadXJUnction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-Curve", "../../Data/roadCurve.osgb");
	Common::AssetLibrary::instance()->loadAsset("TrafficLight", "../../Data/raaTrafficLight.osgb");
	Common::AssetLibrary::instance()->loadAsset("Car-Dumptruck", "../../OpenSceneGraph-Data/dumptruck.osgt");
	Common::AssetLibrary::instance()->loadAsset("Car-Delta", "../../Data/Lancia-Delta.obj");
	Common::AssetLibrary::instance()->loadAsset("Car-Stratos", "../../Data/Lancia-Stratos/source/lshfg4.fbx");

	Common::AssetLibrary::instance()->printAssetList();
	Common::AssetLibrary::instance()->printAsset("Road-Straight");


	g_pRoot = new osg::Group();
	g_pRoot->ref();

	osg::Matrixf mS, mT, mX, mC, mTL, mCarT, mCarS, mCarD;

	mS.makeTranslate(-900.0f, 0.0f, 0.0f);
	mT.makeTranslate(-300.0f, 0.0f, 0.0f);
	mX.makeTranslate(300.0f, 0.0f, 0.0f);
	mC.makeTranslate(900.0f, 0.0f, 0.0f);

	mTL.makeScale(0.2f, 0.2f, 0.2f);

	mCarT = osg::Matrixf::scale(4.0f, 4.0f, 4.0f)*
		osg::Matrixf::rotate(osg::DegreesToRadians(-90.0f), 0.0f, 0.0f, 1.0) *
		osg::Matrixf::translate(120.0f, -150.0f, 20.0f);
	mCarD = osg::Matrixf::scale(40.0f, 40.0f, 40.0f)*
		osg::Matrixf::translate(0.0f, -150.0f, 0.0f);
	mCarS = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 1.0f, 0.0f, 0.0f) *
		osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-120.0f, -150.0f, 0.0f);
		
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mS, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadTJunction", Common::AssetLibrary::instance()->cloneAsset("Road-TJunction"), mT, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadXJunction", Common::AssetLibrary::instance()->cloneAsset("Road-XJunction"), mX, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadCurve", Common::AssetLibrary::instance()->cloneAsset("Road-Curve"), mC, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("TrafficLight", "TrafficLight", Common::AssetLibrary::instance()->getAsset("TrafficLight"), mTL, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("Default", "Car-Dumptruck", Common::AssetLibrary::instance()->getAsset("Car-Dumptruck"), mCarT, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("Car", "Car-Stratos", Common::AssetLibrary::instance()->getAsset("Car-Stratos"), mCarS, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("Default", "Car-Delta", Common::AssetLibrary::instance()->getAsset("Car-Delta"), mCarD, true)->root());

	if(TrafficSystem::RoadFacade *pRF=dynamic_cast<TrafficSystem::RoadFacade*>(Common::Facade::findFacade("RoadStraight")))
	{
		pRF->enableAnimationIDs(true);
		pRF->enableNames(true);
		pRF->enableAnimationPoints(true);

		TrafficSystem::AnimationPointFinder apf;
		apf.find(2, g_pRoot);
		apf.print();
	}

	if (TrafficSystem::CarFacade* pCF = dynamic_cast<TrafficSystem::CarFacade*>(Common::Facade::findFacade("Car-Stratos")))
	{
		// position the detection box for the collider

		osg::Matrixf m = osg::Matrix::translate(-90.0f, -260.0f, 0.0f);

		osg::Vec3f d = osg::Vec3f(100.0f, 100.0f, 30.0f);

		TrafficSystem::Collider* pCollider = dynamic_cast<TrafficSystem::Collider*>(pCF);

		if(pCollider)
		{
			pCollider->setBound(d);
			pCollider->setTransform(m);
		}
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
	viewer.addEventHandler(new ExampleInteractionHandler(keyFunction));


	// set the scene to render
	viewer.setSceneData(g_pRoot);

	// present the 3D window
	viewer.realize();

	// start rendering loop
	return viewer.run();
}
#else

void keyFunction(char c)
{
	switch (c)
	{
		case 'h':
			std::cout << "Simple Viewer - key options" << std::endl;
			std::cout << "\tp - print scene tree to console" << std::endl;
			break;
		case 'p':
			{
				Common::Printer printer;
				printer.traverse(*g_pRoot);
			}
			break;
	}
}



int main(int argc, char* argv[])
{
	osgViewer::Viewer viewer;

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
	viewer.setCamera(pCamera);

	// add manipulators
	viewer.addEventHandler(new osgViewer::ThreadingHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);
	viewer.addEventHandler(new osgViewer::LODScaleHandler);
	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
	viewer.addEventHandler(new ExampleInteractionHandler(keyFunction));

	viewer.setSceneData(g_pRoot);

	// present the 3D window
	viewer.realize();

	// start rendering loop
	return viewer.run();
}

#endif

