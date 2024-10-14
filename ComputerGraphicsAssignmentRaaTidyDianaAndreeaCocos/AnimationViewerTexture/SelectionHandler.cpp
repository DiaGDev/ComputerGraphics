#include "SelectionHandler.h"

SelectionHandler::SelectionHandler(): m_pFacade(0)
{
}

SelectionHandler::SelectionHandler(Common::Facade* pFacade): m_pFacade(pFacade)
{
}

SelectionHandler::~SelectionHandler()
{
}

osg::Object* SelectionHandler::cloneType() const
{
    return new SelectionHandler();
}

osg::Object* SelectionHandler::clone(const osg::CopyOp&) const
{
    return new SelectionHandler(m_pFacade);
}

const char* SelectionHandler::libraryName() const
{
    return "None";
}

const char* SelectionHandler::className() const
{
    return "SelectionHandler";
}

Common::Facade* SelectionHandler::facade()
{
    return m_pFacade;
}
