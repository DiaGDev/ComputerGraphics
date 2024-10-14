#include <windows.h>
#include <iostream>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/NodeTrackerManipulator>

#include "SelectionHandler.h"
#include "DemoTrafficLight.h"
#include "EventHandler.h"

#include "DemoAnimatedCar.h"

EventHandler::EventHandler()
{

}

EventHandler::~EventHandler()
{

}

bool EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
{
    // detect mouse press and setup intersection test
    if (ea.getEventType() & osgGA::GUIEventAdapter::PUSH && ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
    {
        // build an intersector based on the mouse position
        osgUtil::LineSegmentIntersector* pRay = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, ea.getXnormalized(), ea.getYnormalized());
        osgUtil::IntersectionVisitor visitor(pRay);

        //apply the intersection visitor to the scene
		aa.asView()->getCamera()->accept(visitor); 

        if (pRay->containsIntersections()) // if a hit was detected
        {
            //get the first (nearest) hit
            osgUtil::LineSegmentIntersector::Intersection intersection = pRay->getFirstIntersection(); 

            if(intersection.drawable) // check the hit is a drawable
            {
                // search up the drawable's path to find the root of the facade
                // this will be the node we attached the user data to
                for(osg::NodePath::iterator it=intersection.nodePath.begin();it!=intersection.nodePath.end();it++)
                {
                    // if a selection habalder is found
                    if (SelectionHandler* pSH = dynamic_cast<SelectionHandler*>((*it)->getUserData()))
                    {
                        //text to see what type of object facade it is and apply functionallity
                        if (DemoAnimatedCar* pAC = dynamic_cast<DemoAnimatedCar*>(pSH->facade()))
                        {
                            std::cout << "Clicked car" << std::endl;
                        }
                        else if (DemoTrafficLight* pTL = dynamic_cast<DemoTrafficLight*>(pSH->facade()))
                        {   
                            DemoTrafficLight* pDTL = dynamic_cast<DemoTrafficLight*>(Common::Facade::findFacade("PedestrianTrafficLight"));
                            DemoTrafficLight* pDTL1 = dynamic_cast<DemoTrafficLight*>(Common::Facade::findFacade("PedestrianTrafficLight2"));
                            pDTL->toggleState();
                            pDTL1->toggleState();
                        }
                    }
                }
            }
        }
	}

	return false;
}


