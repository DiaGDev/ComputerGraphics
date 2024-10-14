#pragma once

#include <Windows.h>
#include <osg/Node>
#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osg/Observer>


namespace Common
{
	typedef std::map<std::string, class Facade*> FacadeMap;

	class Facade: osg::Observer
	{
	public:
		Facade(std::string sName, osg::Node* pAsset, osg::Matrixf m=osg::Matrixf::identity(), bool bVisible=true);
		virtual ~Facade();

		osg::Node* root();
		osg::Node* asset();
		osg::MatrixTransform* transform();
		bool isVisible();
		void setVisible(bool bVisible);

		static Facade* findFacade(std::string sName);
		virtual void objectDeleted(void*);
		static FacadeMap& facades();

	protected:
		osg::Node* m_pAsset;
		osg::MatrixTransform* m_pTransformation;
		osg::Switch* m_pRoot;
		std::string m_sName;
		bool m_bVisible;

	private:
		static FacadeMap sm_mFacades;
	};

}
