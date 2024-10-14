#include "InteractionHandler.h"

#include <iostream>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>


InteractionHandler::InteractionHandler(KEY_FUNC* pFunc) : m_pFunc(pFunc)
{
}

InteractionHandler::~InteractionHandler()
{
}

bool InteractionHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
{
	switch (ea.getEventType())
	{
	case osgGA::GUIEventAdapter::KEYDOWN:
		std::cout << "The " << ea.getKey() << "was pressed at window position [" << ea.getWindowX() << ", " << ea.getWindowY() << "]" << std::endl;

		if (m_pFunc)
		{
			m_pFunc(ea.getKey());
		}
		else
		{
			std::cout << "There is no specific help for this application" << std::endl;
		}

		if (ea.getKey() == 'h')
		{

			std::cout << std::endl;
			std::cout << "Viewer help - osg key index" << std::endl;
			std::cout << "\tw - switch rendering mode (wireframe/points/filled)" << std::endl;
			std::cout << "\te - toggle frame barrier position" << std::endl;
			std::cout << "\tf - toggle fullscreen" << std::endl;
			std::cout << "\tt - toggle texture state (non shader)" << std::endl;
			std::cout << "\tt - toggle lighting (non shader)" << std::endl;
			std::cout << "\ts - performance display" << std::endl;
			std::cout << "\tm - threading model" << std::endl;
		}


		//    		return true;
	}

	return false;
}
