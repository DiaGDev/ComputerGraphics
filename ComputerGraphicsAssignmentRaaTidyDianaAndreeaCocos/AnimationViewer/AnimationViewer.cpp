#include <windows.h>
#include <iostream>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osg/NodeVisitor>

#include <Common/FacadeManufactory.h>
#include <Common/AssetLibrary.h>
#include <Common/Facade.h>
#include <Common/Printer.h>
#include <Common/NodeFinderT.h>

#include <TrafficSystem/RoadFacadeFactory.h>
#include <TrafficSystem/RoadFacade.h>

#include "DemoAnimatedCar.h"
#include "DemoAnimatedCarFactory.h"
#include "DemoTrafficLight.h"
#include "DemoTrafficLightFactory.h"



osg::Group* g_pRoot;
float g_fTile = 472.0f;

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

float addControlPoint(std::string sTile, std::string sPoint, osg::AnimationPath* pPath, float fTime, float fSpeed, osg::Vec3f& rvLastPos, bool bFirst=false)
{
	//find the facade for the tile we want
	Common::Facade* pF = Common::Facade::findFacade(sTile);

	// setup finder to look for animation point
	Common::NodeFinderT<osg::MatrixTransform> finder(sPoint);


	// use finder to find point in the tile
	if(osg::MatrixTransform *pMT=finder.find(pF->root()))
	{
		// find all the routes between the found anaimation point and the scene root
		// remember there could be many as we have re-used this bit of geometry. We need the one that passes through the root node of this facade
		osg::NodePathList npl = pMT->getParentalNodePaths(0);

		// loop through all of the node paths we have found
		for (osg::NodePathList::iterator it = npl.begin(); it != npl.end();it++)
		{
			// test to see of the current node path is the one that passes through the facade we are considering now
			if(std::find(it->begin(), it->end(), pF->root())!=it->end())
			{
				//we now have the correct route for the animation point we are using

				// calulate the local to world matrix for this route
				osg::Matrix m = osg::computeLocalToWorld(*it);

				//decompose the matrix to get the world translation	and rotation
				osg::Vec3f t, s;
				osg::Quat r, sr;
				m.decompose(t, r, s, sr);

				// if this is not the first control point calculate the time needed to move from the last position
				if(!bFirst)
				{
					float fDistance = (t - rvLastPos).length();
					fTime += fDistance / fSpeed;
				}

				// update the last position - remember this has been passed by reference so we can updated in this function
				rvLastPos = t;

				// add the control point to the animation path
				pPath->insert(fTime, osg::AnimationPath::ControlPoint(t, r));

				// return the updated time 
				return fTime;
			}
		}
	}
	return fTime;
}



int main(int argc, char* argv[])
{
	osgViewer::Viewer viewer;

	g_pRoot = new osg::Group();
	g_pRoot->ref();

	Common::FacadeManufactory::start();
	Common::FacadeManufactory::instance()->addFactory("RoadTile", new TrafficSystem::RoadFacadeFactory());
	Common::FacadeManufactory::instance()->addFactory("TrafficLight", new DemoTrafficLightFactory());
	Common::FacadeManufactory::instance()->addFactory("AnimatedCar", new DemoAnimatedCarFactory());
	TrafficSystem::Collider::toggleVisible();
	Common::AssetLibrary::start();

	Common::AssetLibrary::instance()->loadAsset("Road-Straight", "../../Data/roadStraight.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-TJunction", "../../Data/roadTJunction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-XJunction", "../../Data/roadXJUnction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-Curve", "../../Data/roadCurve.osgb");
	Common::AssetLibrary::instance()->loadAsset("TrafficLight", "../../Data/raaTrafficLight.osgb");
	Common::AssetLibrary::instance()->loadAsset("Car-Delta", "../../Data/Lancia-Delta.obj");


	//setup a simple track
	osg::Matrixf m0, m1, m2, m3;
	 
	m0 = osg::Matrix::translate(-2.0f * g_fTile, 0.0f, 0.0f);
	m1 = osg::Matrix::translate(-1.0f * g_fTile, 0.0f, 0.0f);
	m2 = osg::Matrix::translate(0.0f * g_fTile, 0.0f, 0.0f);
	m3 = osg::Matrix::translate(1.0f * g_fTile, 0.0f, 0.0f);

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "t0", Common::AssetLibrary::instance()->getAsset("Road-Straight"), m0, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "t1", Common::AssetLibrary::instance()->getAsset("Road-Straight"), m1, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "t2", Common::AssetLibrary::instance()->getAsset("Road-Straight"), m2, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "t3", Common::AssetLibrary::instance()->getAsset("Road-Straight"), m3, true)->root());

	/////// Animated Car //////
	//add an animated car
	osg::Matrixf mC = osg::Matrixf::scale(40.0f, 40.0f, 40.0f) *
		osg::Matrix::rotate(osg::DegreesToRadians(90.0f), osg::Vec3f(0.0f, 0.0f, 1.0f)) *
		osg::Matrixf::translate(0.0f, 0.0f, 25.0f);

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("AnimatedCar", "Car", Common::AssetLibrary::instance()->getAsset("Car-Delta"), mC, true)->root());

	//setup animation path
	osg::AnimationPath* pPath=new osg::AnimationPath();
	float fSpeed = 270.0f;
	float fTime = 0.0f;
	osg::Vec3f vLastPos;

	fTime = addControlPoint("t0", "2", pPath, fTime, fSpeed, vLastPos, true);
	fTime = addControlPoint("t0", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("t1", "2", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("t1", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("t2", "2", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("t2", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("t3", "2", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("t3", "0", pPath, fTime, fSpeed, vLastPos);

	// attach animation path to car
	if(DemoAnimatedCar* pAC=dynamic_cast<DemoAnimatedCar*>(Common::Facade::findFacade("Car")))
	{
		pAC->setAnimationPath(pPath);
	}

	//////// collisiony bit ///////
	osg::Matrixf mTL = osg::Matrix::scale(0.03f, 0.03f, 0.03f) * osg::Matrix::rotate(osg::DegreesToRadians(-90.0f), osg::Vec3f(0.0f, 0.0f, 1.0f)) * osg::Matrix::translate(0.0f, 140.0f, 0.0f);

	// add a traffic light - this time very simply to the scene (but the principles are the same as for a controlled junction
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("TrafficLight", "TrafficLight", Common::AssetLibrary::instance()->getAsset("TrafficLight"), mTL, true)->root());

	// the rest of the collision detection is implmented in the DemoAnimatedCar class 'run' method, which is called by the update traversal

	// uncomment this to see the animation points for each tile
	/*
	for(Common::FacadeMap::iterator it=Common::Facade::facades().begin();it!=Common::Facade::facades().end();it++)
	{
		if (TrafficSystem::RoadFacade* pRF = dynamic_cast<TrafficSystem::RoadFacade*>(it->second))
		{
			pRF->enableAnimationIDs(true);
			pRF->enableNames(true);
			pRF->enableAnimationPoints(true);

			// this bit adds the tile name to the billboards over the road tile
			osg::Billboard* pBB = new osg::Billboard();
			pBB->setNormal(osg::Vec3f(0.0f, 0.0f, 1.0f));
			osgText::Text* pT = new osgText::Text();
			pT->setText(it->first);

			pBB->addDrawable(pT);
			pBB->setPosition(0, osg::Vec3f(0.0f, 0.0f, 60.0f));
			pRF->scale()->addChild(pBB);			
		}
	}
	*/

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
//	viewer.addEventHandler(new ExampleInteractionHandler(keyFunction));

	viewer.setSceneData(g_pRoot);

	// present the 3D window
	viewer.realize();

	// start rendering loop
	return viewer.run();
}