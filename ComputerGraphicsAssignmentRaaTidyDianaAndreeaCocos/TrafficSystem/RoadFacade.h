#pragma once

#include <Common/Facade.h>

namespace TrafficSystem
{
	class RoadFacade: public Common::Facade
	{
	public:
		RoadFacade(std::string sName, osg::Node* pAsset, osg::Matrixf m = osg::Matrixf::identity(), bool bVisible = true);
		virtual ~RoadFacade();

		osg::MatrixTransform* translate();
		osg::MatrixTransform* rotate();
		osg::MatrixTransform* scale();

		void enableNames(bool bEnable);
		void enableAnimationPoints(bool bEnable);
		void enableAnimationIDs(bool bEnable);

	protected:
		osg::MatrixTransform* m_pRotate;
		osg::MatrixTransform* m_pScale;
		//Note: translate is the parent transform object

		osg::Switch* m_pNames;
		osg::Switch* m_pAnimationPoints;
		osg::Switch* m_pAnimationIDs;
	};
}
