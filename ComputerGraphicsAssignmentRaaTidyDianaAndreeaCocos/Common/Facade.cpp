#include "Facade.h"

Common::FacadeMap Common::Facade::sm_mFacades;


Common::Facade::Facade(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible): m_sName(sName), m_pAsset(pAsset), m_bVisible(bVisible), m_pRoot(new osg::Switch()), m_pTransformation(new osg::MatrixTransform)
{
	if(sName.length() && m_pAsset)
	{
		m_pAsset->ref();
		m_pAsset->addObserver(this);

		m_pRoot->ref();
		m_pRoot->setName(sName);
		m_pRoot->addObserver(this);
		if (m_bVisible) m_pRoot->setAllChildrenOn(); else m_pRoot->setAllChildrenOff();

		m_pTransformation->ref();
		m_pTransformation->setName("Transformation");
		m_pTransformation->setMatrix(m);

		m_pRoot->addChild(m_pTransformation);
		m_pTransformation->addChild(m_pAsset);

		sm_mFacades[sName] = this;
	}
}

Common::Facade::~Facade()
{
	if (m_pAsset) m_pAsset->unref();
	if (m_pRoot) m_pRoot->unref();
	if (m_pTransformation) m_pTransformation->unref();
	sm_mFacades.erase(m_sName);
}

osg::Node* Common::Facade::root()
{
	return m_pRoot;
}

osg::Node* Common::Facade::asset()
{
	return m_pAsset;
}

osg::MatrixTransform* Common::Facade::transform()
{
	return m_pTransformation;
}

bool Common::Facade::isVisible()
{
	return m_bVisible;
}

void Common::Facade::setVisible(bool bVisible)
{
	if (m_bVisible != bVisible)
	{
		m_bVisible = bVisible;
		if (m_pRoot)
		{
			if (m_bVisible) m_pRoot->setAllChildrenOn();
			else m_pRoot->setAllChildrenOff();
		}
	}
}

Common::Facade* Common::Facade::findFacade(std::string sName)
{
	if (sName.length() && sm_mFacades.find(sName) != sm_mFacades.end()) return sm_mFacades[sName];
	return 0;
}

void Common::Facade::objectDeleted(void*)
{
	delete this;
}

Common::FacadeMap& Common::Facade::facades()
{
	return sm_mFacades;
}
