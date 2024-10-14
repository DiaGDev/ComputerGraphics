#include "FacadeManufactory.h"
#include "Facade.h"
#include "FacadeFactory.h"

Common::FacadeManufactory* Common::FacadeManufactory::sm_pManufactory=0;

void Common::FacadeManufactory::start()
{
    if (!sm_pManufactory) new FacadeManufactory();
}

void Common::FacadeManufactory::end()
{
    if (sm_pManufactory) delete sm_pManufactory;
}

Common::FacadeManufactory* Common::FacadeManufactory::instance()
{
    start();
    return sm_pManufactory;
}

bool Common::FacadeManufactory::addFactory(std::string sName, FacadeFactoryA* pFactory)
{
    bool bRet = false;
    if(sName.length() && m_mFactories.find(sName)==m_mFactories.end() && pFactory)
    {
        m_mFactories[sName] = pFactory;
        bRet=true;
    }
    return bRet;
}

Common::Facade* Common::FacadeManufactory::create(std::string sFacadeFactory, std::string sName, osg::Node* pNode, osg::Matrixf mTrans, bool bVisible)
{
    Facade* pFacade = 0;

    if(sFacadeFactory.length() && sName.length() && pNode)
    {
        if(m_mFactories.find(sFacadeFactory) != m_mFactories.end() && !Facade::findFacade(sName))
	        pFacade = m_mFactories[sFacadeFactory]->create(sName, pNode, mTrans, bVisible);
        else
            pFacade = m_mFactories["Default"]->create(sName, pNode, mTrans, bVisible);
    }

	return pFacade;
}

Common::FacadeManufactory::FacadeManufactory()
{
    if (!sm_pManufactory) sm_pManufactory = this;

    m_mFactories["Default"] = new FacadeFactory();

}

Common::FacadeManufactory::~FacadeManufactory()
{
    if (sm_pManufactory == this) sm_pManufactory = 0;

	for(FacadeFactories::iterator it=m_mFactories.begin();it!=m_mFactories.end();it++)
        delete (*it).second;
    m_mFactories.clear();
}
