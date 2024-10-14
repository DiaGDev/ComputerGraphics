#include <windows.h>
#include <iostream>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/NodeVisitor>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

#include <Common/FacadeManufactory.h>
#include <Common/AssetLibrary.h>
#include <Common/Printer.h>
#include <Common/NodeFinderT.h>
#include <Common/Facade.h>

#include <TrafficSystem/TrafficLightFacade.h>
#include <TrafficSystem/TrafficLightFacadeFactory.h>
#include <TrafficSystem/CarFacadeFactory.h>
#include <TrafficSystem/Collider.h>
#include <TrafficSystem/RoadFacadeFactory.h>
#include <TrafficSystem/RoadFacade.h>

#include <Assignment/RoadTileLightsFacadeFactory.h>
#include <Assignment/RoadTileLightsFacade.h>
#include <Assignment/ControllableTrafficLightFacade.h>
#include <Assignment/ControllableTrafficLightFacadeFactory.h>

#include "DemoAnimatedCar.h"
#include "DemoAnimatedCarFactory.h"
#include "DemoTrafficLight.h"
#include "DemoTrafficLightFactory.h"

#include "EventHandler.h"


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
	Common::FacadeManufactory::instance()->addFactory("RoadTile", new Assignment::RoadTileLightsFacadeFactory());
	Common::FacadeManufactory::instance()->addFactory("TrafficLight", new Assignment::ControllableTrafficLightFacadeFactory());
	Common::FacadeManufactory::instance()->addFactory("PedestrianTrafficLight", new DemoTrafficLightFactory());
	Common::FacadeManufactory::instance()->addFactory("AnimatedCar", new DemoAnimatedCarFactory());
	TrafficSystem::Collider::toggleVisible();
	Common::AssetLibrary::start();

	Common::AssetLibrary::instance()->loadAsset("Road-Straight", "../../Data/roadStraight.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-TJunction", "../../Data/roadTJunction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-XJunction", "../../Data/roadXJUnction.osgb");
	Common::AssetLibrary::instance()->loadAsset("Road-Curve", "../../Data/roadCurve.osgb");
	Common::AssetLibrary::instance()->loadAsset("TrafficLight", "../../Data/raaTrafficLight.osgb");
	Common::AssetLibrary::instance()->loadAsset("Car-Delta", "../../Data/Lancia-Delta.obj");
	Common::AssetLibrary::instance()->loadAsset("Car-Dumptruck", "../../OpenSceneGraph-Data/dumptruck.osgt");
	Common::AssetLibrary::instance()->loadAsset("Car-Stratos", "../../Data/Lancia-Stratos/source/lshfg4.fbx");

	//tiles matrix
	osg::Matrixf mRC_00, mRC_40, mRC_09, mRC_49,
		mRT_20, mRT_03, mRT_06, mRT_29, mRT_46, mRT_43,
		mRX_26, mRX_23,
		mRS_10, mRS_30, mRS_13, mRS_19, mRS_33, mRS_16, mRS_36, mRS_39, mRS_21, mRS_22, mRS_24, mRS_25, mRS_27, mRS_28,
		mRS_01, mRS_02, mRS_04, mRS_05, mRS_07, mRS_08, mRS_41, mRS_42, mRS_44, mRS_45, mRS_47, mRS_48;

	//Starting from first Vertical X Junction
	// Insides
	//X junctions
	mRX_26.makeTranslate(0.0f, 0.0f, 0.0f);
	mRX_23.makeTranslate(0.0f, -472.0f * 3, 0.0f);

	//TJunctions
	mRT_20 = osg::Matrixf::rotate(osg::DegreesToRadians(270.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, -472.0f * 6, 0.0f);
	mRT_29 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, 472.0f * 3, 0.0f);
	mRT_06 = osg::Matrixf::rotate(osg::DegreesToRadians(180.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, 0.0f, 0.0f);
	mRT_03 = osg::Matrixf::rotate(osg::DegreesToRadians(180.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, -472.0f * 3, 0.0f);
	mRT_46.makeTranslate(472.0f * 2, 0.0f, 0.0f);
	mRT_43.makeTranslate(472.0f * 2, -472.0f * 3, 0.0f);

	//Corners
	mRC_09 = osg::Matrixf::rotate(osg::DegreesToRadians(270.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, 472.0f * 3, 0.0f);
	mRC_00.makeTranslate(-472.0f * 2, -472.0f * 6, 0.0f);
	mRC_49 = osg::Matrixf::rotate(osg::DegreesToRadians(180.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, 472.0f * 3, 0.0f);
	mRC_40 = osg::Matrixf::rotate(osg::DegreesToRadians(-270.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, -472.0f * 6, 0.0f);

	//Straight Roads H
	mRS_16.makeTranslate(-472.0f, 0.0f, 0.0f);
	mRS_36.makeTranslate(472.0f, 0.0f, 0.0f);
	mRS_13.makeTranslate(-472.0f, -472.0f * 3, 0.0f);
	mRS_33.makeTranslate(472.0f, -472.0f * 3, 0.0f);

	mRS_10.makeTranslate(-472.0f, -472.0f * 6, 0.0f);
	mRS_30.makeTranslate(472.0f, -472.0f * 6, 0.0f);
	mRS_19.makeTranslate(-472.0f, 472.0f * 3, 0.0f);
	mRS_39.makeTranslate(472.0f, 472.0f * 3, 0.0f);

	//Straight Roads V
	mRS_27 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, 472.0f, 0.0f);
	mRS_28 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, 472.0f * 2, 0.0f);
	mRS_25 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, -472.0f, 0.0f);
	mRS_24 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, -472.0f * 2, 0.0f);
	mRS_22 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, -472.0f * 4, 0.0f);
	mRS_21 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(0.0f, -472.0f * 5, 0.0f);

	mRS_08 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, 472.0f * 2, 0.0f);
	mRS_07 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, 472.0f, 0.0f);
	mRS_05 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, -472.0f, 0.0f);
	mRS_04 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, -472.0f * 2, 0.0f);
	mRS_02 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, -472.0f * 4, 0.0f);
	mRS_01 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-472.0f * 2, -472.0f * 5, 0.0f);

	mRS_48 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, 472.0f * 2, 0.0f);
	mRS_47 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, 472.0f, 0.0f);
	mRS_45 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, -472.0f, 0.0f);
	mRS_44 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, -472.0f * 2, 0.0f);
	mRS_42 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, -472.0f * 4, 0.0f);
	mRS_41 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(472.0f * 2, -472.0f * 5, 0.0f);

	//Pedestrian traffic lights
	osg::Matrixf mPTL, mPTL2;
	mPTL= osg::Matrix::scale(0.03f, 0.03f, 0.03f) * osg::Matrix::rotate(osg::DegreesToRadians(180.0f), osg::Vec3f(0.0f, 0.0f, 1.0f)) * osg::Matrix::translate(472.0f * 2 + 140, 472.0f, 0.0f);
	mPTL2 = osg::Matrix::scale(0.03f, 0.03f, 0.03f) * osg::Matrix::rotate(osg::DegreesToRadians(0.0f), osg::Vec3f(0.0f, 0.0f, 1.0f)) * osg::Matrix::translate(472.0f * 2 - 140, 472.0f, 0.0f);

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("PedestrianTrafficLight", "PedestrianTrafficLight", Common::AssetLibrary::instance()->getAsset("TrafficLight"), mPTL, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("PedestrianTrafficLight", "PedestrianTrafficLight2", Common::AssetLibrary::instance()->getAsset("TrafficLight"), mPTL2, true)->root());


	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadXJunction_23", Common::AssetLibrary::instance()->cloneAsset("Road-XJunction"), mRX_23, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadXJunction_26", Common::AssetLibrary::instance()->cloneAsset("Road-XJunction"), mRX_26, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadTJunction_20", Common::AssetLibrary::instance()->cloneAsset("Road-TJunction"), mRT_20, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadTJunction_29", Common::AssetLibrary::instance()->cloneAsset("Road-TJunction"), mRT_29, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadTJunction_06", Common::AssetLibrary::instance()->cloneAsset("Road-TJunction"), mRT_06, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadTJunction_03", Common::AssetLibrary::instance()->cloneAsset("Road-TJunction"), mRT_03, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadTJunction_46", Common::AssetLibrary::instance()->cloneAsset("Road-TJunction"), mRT_46, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadTJunction_43", Common::AssetLibrary::instance()->cloneAsset("Road-TJunction"), mRT_43, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadCurve_09", Common::AssetLibrary::instance()->cloneAsset("Road-Curve"), mRC_09, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadCurve_00", Common::AssetLibrary::instance()->cloneAsset("Road-Curve"), mRC_00, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadCurve_49", Common::AssetLibrary::instance()->cloneAsset("Road-Curve"), mRC_49, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadCurve_40", Common::AssetLibrary::instance()->cloneAsset("Road-Curve"), mRC_40, true)->root());

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_16", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_16, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_36", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_36, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_13", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_13, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_33", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_33, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_10", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_10, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_30", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_30, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_19", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_19, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_39", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_39, true)->root());

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_27", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_27, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_28", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_28, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_25", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_25, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_24", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_24, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_22", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_22, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_21", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_21, true)->root());

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_08", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_08, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_07", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_07, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_05", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_05, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_04", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_04, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_02", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_02, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_01", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_01, true)->root());

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_48", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_48, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_47", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_47, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_45", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_45, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_44", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_44, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_42", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_42, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("RoadTile", "RoadStraight_41", Common::AssetLibrary::instance()->cloneAsset("Road-Straight"), mRS_41, true)->root());

	//car creation
	osg::Matrixf mCD1, mCS1, mCT1;
	mCD1 = osg::Matrixf::scale(40.0f, 40.0f, 40.0f) *
		osg::Matrix::rotate(osg::DegreesToRadians(90.0f), osg::Vec3f(0.0f, 0.0f, 1.0f)) *
		osg::Matrixf::translate(-15.0f, 0.0f, 25.0f);

	mCS1 = osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 1.0f, 0.0f, 0.0f) *
		osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) *
		osg::Matrixf::translate(-15.0f, 0.0f, 0.0f);

	mCT1 = osg::Matrixf::scale(8.0f, 8.0f, 8.0f) *
		osg::Matrixf::translate(80.0f, -15.0f, 47.0f);

	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("AnimatedCar", "CarDelta1", Common::AssetLibrary::instance()->getAsset("Car-Delta"), mCD1, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("AnimatedCar", "CarStratos1", Common::AssetLibrary::instance()->getAsset("Car-Stratos"), mCS1, true)->root());
	g_pRoot->addChild(Common::FacadeManufactory::instance()->create("AnimatedCar", "CarDumptruck1", Common::AssetLibrary::instance()->getAsset("Car-Dumptruck"), mCT1, true)->root());


	//Yellow car
	osg::AnimationPath* pPath = new osg::AnimationPath();
	float fSpeed = 200.0f;
	float fTime = 0.0f;
	osg::Vec3f vLastPos;

	fTime = addControlPoint("RoadStraight_24", "2", pPath, fTime, fSpeed, vLastPos, true); 
	fTime = addControlPoint("RoadStraight_24", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_25", "2", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_25", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadXJunction_26", "8", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadXJunction_26", "9", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadXJunction_26", "7", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_16", "1", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_16", "3", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadTJunction_06", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadTJunction_06", "1", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadTJunction_06", "2", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_05", "1", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_05", "3", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_04", "1", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_04", "3", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadTJunction_03", "8", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadTJunction_03", "9", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadTJunction_03", "7", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_13", "2", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_13", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadXJunction_23", "0", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadXJunction_23", "1", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadXJunction_23", "2", pPath, fTime, fSpeed, vLastPos);
	fTime = addControlPoint("RoadStraight_24", "2", pPath, fTime, fSpeed, vLastPos);

	if (DemoAnimatedCar* pAC = dynamic_cast<DemoAnimatedCar*>(Common::Facade::findFacade("CarDelta1")))
	{
		pAC->setTransform(osg::Matrixf::translate(1.0f, -2.5f, -2.5f));
		pAC->setBound(osg::Vec3f(4.0f, 2.0f, 5.0f));
		pAC->setAnimationPath(pPath);
	}

	//Blue Car
	osg::AnimationPath* pPath2 = new osg::AnimationPath();
	float fSpeed2 = 280.0f;
	float fTime2 = 0.0f;
	osg::Vec3f vLastPos2;

	fTime2 = addControlPoint("RoadStraight_47", "1", pPath2, fTime2, fSpeed2, vLastPos2, true);
	fTime2 = addControlPoint("RoadStraight_47", "3", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadTJunction_46", "5", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadTJunction_46", "6", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadTJunction_46", "7", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_36", "1", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_36", "3", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadXJunction_26", "11", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadXJunction_26", "13", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadXJunction_26", "2", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_27", "2", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_27", "0", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_28", "2", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_28", "0", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadTJunction_29", "0", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadTJunction_29", "3", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadTJunction_29", "4", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_39", "2", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_39", "0", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadCurve_49", "3", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadCurve_49", "4", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadCurve_49", "5", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_48", "1", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_48", "3", pPath2, fTime2, fSpeed2, vLastPos2);
	fTime2 = addControlPoint("RoadStraight_47", "1", pPath2, fTime2, fSpeed2, vLastPos2);



	if (DemoAnimatedCar* pAC = dynamic_cast<DemoAnimatedCar*>(Common::Facade::findFacade("CarStratos1")))
	{
		pAC->setTransform(osg::Matrixf::translate(-40.0f, 90.0f, 5.0f)*
			osg::Matrixf::rotate(osg::DegreesToRadians(-90.0f), 1.0f, 0.0f, 0.0f)*
		osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 1.0f, 0.0f));
		pAC->setBound(osg::Vec3f(160.0f, 80.0f, 50.0f));
		pAC->setAnimationPath(pPath2);
	}

	//Dumptruck Car
	osg::AnimationPath* pPath3 = new osg::AnimationPath();
	float fSpeed3 = 300.0f;
	float fTime3 = 0.0f;
	osg::Vec3f vLastPos3;


	fTime3 = addControlPoint("RoadStraight_41", "2", pPath3, fTime3, fSpeed3, vLastPos3,true); 
	fTime3 = addControlPoint("RoadStraight_41", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_42", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_42", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_43", "8", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_43", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_44", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_44", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_45", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_45", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_46", "8", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_46", "9", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_46", "7", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_36", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_36", "3", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadXJunction_26", "11", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadXJunction_26", "7", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_16", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_16", "3", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_06", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_06", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_06", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_05", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_05", "3", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_04", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_04", "3", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_03", "8", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_03", "9", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_03", "7", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_13", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_13", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadXJunction_23", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadXJunction_23", "3", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadXJunction_23", "4", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_22", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_22", "3", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_21", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_21", "3", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_20", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_20", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadTJunction_20", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_30", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_30", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadCurve_40", "0", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadCurve_40", "1", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadCurve_40", "2", pPath3, fTime3, fSpeed3, vLastPos3);
	fTime3 = addControlPoint("RoadStraight_41", "2", pPath3, fTime3, fSpeed3, vLastPos3);

	if (DemoAnimatedCar* pAC = dynamic_cast<DemoAnimatedCar*>(Common::Facade::findFacade("CarDumptruck1")))
	{
		pAC->setTransform(osg::Matrixf::translate(11.0f, 8.0f, -4.5f) *
			osg::Matrixf::rotate(osg::DegreesToRadians(0.0f), 1.0f, 0.0f, 0.0f));
		pAC->setBound(osg::Vec3f(10.0f, 20.0f, 4.0f));
		pAC->setAnimationPath(pPath3);
	}

	if (Common::Facade* pDumpTruck = Common::Facade::findFacade("CarDumptruck1"))
	{
		osg::Shader* pVertShader = new osg::Shader(osg::Shader::VERTEX);
		pVertShader->loadShaderSourceFromFile("../../shaders/colour.vert");
		osg::Shader* pFragShader = new osg::Shader(osg::Shader::FRAGMENT);
		pFragShader->loadShaderSourceFromFile("../../shaders/colour.frag");
		osg::Program* pShaderProgram = new osg::Program();
		pShaderProgram->addShader(pVertShader);
		pShaderProgram->addShader(pFragShader);
		pDumpTruck->asset()->getOrCreateStateSet()->setAttributeAndModes(pShaderProgram,
			osg::StateAttribute::ON);
		osg::Uniform* pStateUniform = new osg::Uniform(osg::Uniform::INT, "state");
		pStateUniform->set(0);
		pDumpTruck->asset()->getOrCreateStateSet()->addUniform(pStateUniform);
	}


	if (Assignment::RoadTileLightsFacade* pRTL = dynamic_cast<Assignment::RoadTileLightsFacade*>(Common::Facade::findFacade("RoadXJunction_26")))
	{
		osg::Matrix mX0, mX1, mX2, mX3;

		mX0 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(-90.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(-190.0f, 160.0f, 0.0f);
		mX1 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(0.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(-170.0f, -190.0f, 0.0f);
		mX2 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(90.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(170.0f, -160.0f, 0.0f);
		mX3 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(180.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(170.0f, 160.0f, 0.0f);

	
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "XTL0", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), mX0, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "XTL1", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), mX1, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "XTL2", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), mX2, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "XTL3", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), mX3, true)));

	}

	if (Assignment::RoadTileLightsFacade* pRTL = dynamic_cast<Assignment::RoadTileLightsFacade*>(Common::Facade::findFacade("RoadTJunction_03")))
	{
		osg::Matrix mT0, mT1, mT2;

		mT0 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(-90.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(-170.0f, 160.0f, 0.0f);
		mT1 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(0.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(-170.0f, -160.0f, 0.0f);
		mT2 = osg::Matrixf::scale(0.03f, 0.03f, 0.03f) * osg::Matrixf::rotate(osg::DegreesToRadians(180.0f), 0.0f, 0.0f, 1.0f) * osg::Matrixf::translate(170.0f, 160.0f, 0.0f);

		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "TTL0", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), mT0, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "TTL1", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), mT1, true)));
		pRTL->addLight(dynamic_cast<Assignment::ControllableTrafficLightFacade*>(Common::FacadeManufactory::instance()->create("TrafficLight", "TTL3", Common::AssetLibrary::instance()->cloneAsset("TrafficLight"), mT2, true)));

	}

	/*for (Common::FacadeMap::iterator it = Common::Facade::facades().begin(); it != Common::Facade::facades().end(); it++)
	{
		if (TrafficSystem::RoadFacade* pRF = dynamic_cast<TrafficSystem::RoadFacade*>(it->second))
		{
			//pRF->enableAnimationIDs(true);
			//pRF->enableNames(true);
			//pRF->enableAnimationPoints(true);

			// this bit adds the tile name to the billboards over the road tile
			osg::Billboard* pBB = new osg::Billboard();
			pBB->setNormal(osg::Vec3f(0.0f, 0.0f, 1.0f));
			osgText::Text* pT = new osgText::Text();
			pT->setText(it->first);

			pBB->addDrawable(pT);
			pBB->setPosition(0, osg::Vec3f(0.0f, 0.0f, 60.0f));
			pRF->scale()->addChild(pBB);
		}
	}*/

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
	viewer.addEventHandler(new EventHandler());

	viewer.setSceneData(g_pRoot);

	// present the 3D window
	viewer.realize();

	// start rendering loop
	return viewer.run();

/////////OriginalScriptStuff/////////////
	/*//setup a simple track
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

	// the rest of the collision detection is implmented in the DemoAnimatedCar class 'run' method, which is called by the update traversal*/


}