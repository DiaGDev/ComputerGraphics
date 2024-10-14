#pragma once

#include <Windows.h>
#include <string>
#include <osg/Node>
#include <osg/NodeVisitor>

namespace Common
{
	typedef std::map<std::string, osg::Node*> AssetMap;

	class AssetLibrary
	{
	public:
		static void start();
		static void end();
		static AssetLibrary* instance();

		void loadAsset(std::string sName, std::string sFile);
		void removeAsset(std::string sName);
		void clear();
		osg::Node* getAsset(std::string sName);
		osg::Node* cloneAsset(std::string sName);

		void printAssetList();
		void printAsset(std::string sName);

	protected:
		AssetLibrary();
		virtual ~AssetLibrary();

	private:
		static AssetLibrary* sm_pInstance;
		AssetMap m_mAssets;

		// Note: the AssetFixer corrects a lighting problem that can occur with some objects when loaded by switching on
		// GL_NORMALISE for all the osg::Geode objects in the loaded file
		class AssetFixer: osg::NodeVisitor
		{
		public:
			AssetFixer(osg::Node* pNode);

			virtual void apply(osg::Geode& rGeode);
			virtual void apply(osg::Node& rNode);


		};
	};
}
