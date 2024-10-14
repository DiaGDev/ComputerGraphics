#include <windows.h>
#include <osgDB/ReadFile>

#include "AssetLibrary.h"

#include "Printer.h"

Common::AssetLibrary* Common::AssetLibrary::sm_pInstance=0;

void Common::AssetLibrary::start()
{
	if (!sm_pInstance) new AssetLibrary();
}

void Common::AssetLibrary::end()
{
	if (sm_pInstance) delete sm_pInstance;
} 
 
Common::AssetLibrary* Common::AssetLibrary::instance()
{
	start();
	return sm_pInstance;
}

void Common::AssetLibrary::loadAsset(std::string sName, std::string sFile)
{
	if (sName.length() && sFile.length() && m_mAssets.find(sName) == m_mAssets.end())
	{
		osg::Node* pAsset = osgDB::readNodeFile(sFile);
		if (pAsset)
		{
			m_mAssets[sName] = pAsset;
			pAsset->ref();
			AssetFixer af(pAsset);

		}
	}
}

void Common::AssetLibrary::removeAsset(std::string sName)
{
	if (sName.length() && m_mAssets.find(sName) != m_mAssets.end())
	{
		m_mAssets[sName]->unref();
		m_mAssets.erase(sName);
	}
}

void Common::AssetLibrary::clear()
{
	for (AssetMap::iterator it = m_mAssets.begin(); it != m_mAssets.end(); it++) it->second->unref();
	m_mAssets.clear();
}

osg::Node* Common::AssetLibrary::getAsset(std::string sName)
{
	osg::Node* pAsset = 0;

	if (sName.length() && m_mAssets.find(sName) != m_mAssets.end()) pAsset = m_mAssets[sName];

	return pAsset;
}

osg::Node* Common::AssetLibrary::cloneAsset(std::string sName)
{
	osg::Node* pAsset = 0;

	if (sName.length() && m_mAssets.find(sName) != m_mAssets.end()) pAsset = dynamic_cast<osg::Node*>(m_mAssets[sName]->clone(osg::CopyOp::DEEP_COPY_ALL));

	return pAsset;
}

void Common::AssetLibrary::printAssetList()
{
	std::cout << "Asset List" << std::endl;
	for (AssetMap::iterator it = m_mAssets.begin(); it != m_mAssets.end(); it++)
		std::cout << "\t" << it->first << (it->second ? " [Loaded]" : " [Not Loaded]") << std::endl;
	std::cout << "Asset List End" << std::endl;
}

void Common::AssetLibrary::printAsset(std::string sName)
{
	if (m_mAssets.find(sName) != m_mAssets.end())
	{
		if (m_mAssets[sName])
		{
			Printer p;
			p.traverse(*m_mAssets[sName]);
		}
		else
			std::cout << "Print Asset [" << sName << "] not loaded" << std::endl;
	}
	else
		std::cout << "Print Asset [" << sName << "] not found" << std::endl;
}

Common::AssetLibrary::AssetLibrary()
{
	if (!sm_pInstance) sm_pInstance = this;
}

Common::AssetLibrary::~AssetLibrary()
{
	if (sm_pInstance == this) sm_pInstance = 0;
	clear();
}

Common::AssetLibrary::AssetFixer::AssetFixer(osg::Node* pNode) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
	if (pNode) traverse(*pNode);
}

void Common::AssetLibrary::AssetFixer::apply(osg::Geode& rGeode)
{
	rGeode.getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON || osg::StateAttribute::OVERRIDE);
	traverse(rGeode);
}

void Common::AssetLibrary::AssetFixer::apply(osg::Node& rNode)
{
	traverse(rNode);
}
