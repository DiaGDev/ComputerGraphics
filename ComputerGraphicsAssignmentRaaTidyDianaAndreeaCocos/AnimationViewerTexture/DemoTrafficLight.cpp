#include <windows.h>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osg/Array>

#include <Common/NodeFinderT.h>

#include "SelectionHandler.h"
#include "DemoTrafficLight.h"

#include <osg/Texture2D>

osg::Material* DemoTrafficLight::sm_pStopMaterial=0;
osg::Material* DemoTrafficLight::sm_pGoMaterial = 0;

DemoTrafficLight::DemoTrafficLight(std::string sName, osg::Node* pAsset, osg::Matrixf m, bool bVisible): TrafficLightFacade(sName, pAsset,m, bVisible), m_pSelectTransform(new osg::MatrixTransform()), m_pRedLamp(0), m_pGreenLamp(0), m_pAmberLamp(0)
{
    // setup materials for stop and go
    if (!sm_pGoMaterial)
    {
        sm_pGoMaterial = new osg::Material();
        sm_pGoMaterial->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4f(0.1f, 0.3f, 0.1f, 1.0f));
        sm_pGoMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4f(0.4f, 0.8f, 0.5f, 1.0f));
        sm_pGoMaterial->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4f(0.1f, 0.1f, 0.1f, 1.0f));
    }
    sm_pGoMaterial->ref();

	if (!sm_pStopMaterial)
    {
        sm_pStopMaterial = new osg::Material();
        sm_pStopMaterial->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4f(0.3f, 0.1f, 0.1f, 1.0f));
        sm_pStopMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4f(0.8f, 0.2f, 0.2f, 1.0f));
        sm_pStopMaterial->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4f(0.1f, 0.1f, 0.1f, 1.0f));
    }
    sm_pStopMaterial->ref();

    // load the texture to a texture object - and set the min/mag filters
    osg::Texture2D* pTexture = new osg::Texture2D(osgDB::readImageFile("../../Data/walk.jpg"));
    pTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_NEAREST);
    pTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);

    // define the geometry for the simple quad - these will be used in slightly different ways in the textureing approaches
    osg::Vec3Array* pvVerts = new osg::Vec3Array();
    pvVerts->push_back(osg::Vec3f(-275.0f, 0.0f, -275.0f));
    pvVerts->push_back(osg::Vec3f(275.0f, 0.0f, -275.0f));
    pvVerts->push_back(osg::Vec3f(275.0f, 0.0f, 275.0f));
    pvVerts->push_back(osg::Vec3f(-275.0f, 0.0f, 275.0f));

    osg::Vec3Array* pvNorms = new osg::Vec3Array();
    pvNorms->push_back(osg::Vec3f(0.0f, 1.0f, 0.0f));

    osg::Vec2Array* pvTex0 = new osg::Vec2Array();
    pvTex0->push_back(osg::Vec2f(0.33f, 0.52f));
    pvTex0->push_back(osg::Vec2f(0.65f, 0.52f));
    pvTex0->push_back(osg::Vec2f(0.65f, 0.89f));
    pvTex0->push_back(osg::Vec2f(0.33f, 0.89f));

	osg::Vec2Array* pvTex1 = new osg::Vec2Array();
    pvTex1->push_back(osg::Vec2f(0.33f, 0.11f));
    pvTex1->push_back(osg::Vec2f(0.65f, 0.11f));
    pvTex1->push_back(osg::Vec2f(0.65f, 0.48f));
    pvTex1->push_back(osg::Vec2f(0.33f, 0.48f));

    // define the transformation for the geomtry
    osg::MatrixTransform* pTexTransform = new osg::MatrixTransform(osg::Matrix::rotate(osg::DegreesToRadians(-90.0f), osg::Vec3f(0.0f, 0.0f, 1.0f)) * osg::Matrix::translate(osg::Vec3f(0.0f, 0.0f, 5000.0f)));


#ifdef WALK_SHADER
    //In this case we will use a shader programme to 'cutout' the person form the image and add some effects

    // load the shader code (this is in /shaders)
	osg::Shader* pWalkVert = new osg::Shader(osg::Shader::VERTEX);
    pWalkVert->loadShaderSourceFromFile("../../shaders/walk.vert");

    osg::Shader* pWalkFrag = new osg::Shader(osg::Shader::FRAGMENT);
    pWalkFrag->loadShaderSourceFromFile("../../shaders/walk.frag");

    // compile the shader programme
    osg::Program* pWalkProg = new osg::Program();
    pWalkProg->addShader(pWalkVert);
    pWalkProg->addShader(pWalkFrag);

    // find the shader unifrom values for apssing data
    m_pWalkStateUniform = new osg::Uniform(osg::Uniform::INT, "walkState");
    m_pWalkTimeUniform = new osg::Uniform(osg::Uniform::FLOAT, "walkTime");

    // build the geometry
    // in this case is is much simpler than the non-shader form as we will use the shader to control the texture and
    // do not need two geometries for rendering the different states
    // notice how we are now providing one texture and two sets of texture coordinates to the geometry

	osg::Geode* pTexGeode = new osg::Geode();
    pTexGeode->getOrCreateStateSet()->setAttributeAndModes(pWalkProg, osg::StateAttribute::ON);
    pTexGeode->getOrCreateStateSet()->addUniform(m_pWalkStateUniform);
    pTexGeode->getOrCreateStateSet()->addUniform(m_pWalkTimeUniform);
    m_pWalkStateUniform->set(0);

    osg::Geometry* pQuad = new osg::Geometry();
    pQuad->setVertexArray(pvVerts);
    pQuad->setNormalArray(pvNorms);
    pQuad->setNormalBinding(osg::Geometry::BIND_OVERALL);
    pQuad->setTexCoordArray(0, pvTex0);
    pQuad->setTexCoordArray(1, pvTex1);
    pQuad->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    m_pTransformation->addChild(pTexTransform);
    pTexTransform->addChild(pTexGeode);
	pTexGeode->addDrawable(pQuad);

    // add the texture to the sahpe
	pTexGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, pTexture);

    //setup the callback for the timer function - glow effect
	pTexGeode->setUpdateCallback(this);

#else
    // in this simpler for we create two geometries and switch between them with a section of the texture on each
    osg::Geode* pTexGeode0 = new osg::Geode();
    osg::Geode* pTexGeode1 = new osg::Geode();

    osg::Geometry* pQuad0 = new osg::Geometry();
    pQuad0->setVertexArray(pvVerts);
    pQuad0->setNormalArray(pvNorms);
    pQuad0->setNormalBinding(osg::Geometry::BIND_OVERALL);
    pQuad0->setTexCoordArray(0, pvTex0);
    pQuad0->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

	osg::Geometry* pQuad1 = new osg::Geometry();
    pQuad1->setVertexArray(pvVerts);
    pQuad1->setNormalArray(pvNorms);
    pQuad1->setNormalBinding(osg::Geometry::BIND_OVERALL);
    pQuad1->setTexCoordArray(0, pvTex1);
    pQuad1->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    m_pTextureSwitch = new osg::Switch();
	m_pTransformation->addChild(pTexTransform);
    pTexTransform->addChild(m_pTextureSwitch);
	m_pTextureSwitch->addChild(pTexGeode0);
    m_pTextureSwitch->addChild(pTexGeode1);
    pTexGeode0->addDrawable(pQuad0);
    pTexGeode1->addDrawable(pQuad1);

    pTexGeode0->getOrCreateStateSet()->setTextureAttributeAndModes(0, pTexture);
    pTexGeode1->getOrCreateStateSet()->setTextureAttributeAndModes(0, pTexture);

    m_pTextureSwitch->setSingleChildOn(0);

#endif

    // find geometry for lamp colours
    Common::NodeFinderT<osg::Geode> amberFinder("trafficLight::AmberLamp-GEODE");
    Common::NodeFinderT<osg::Geode> redFinder("trafficLight::RedLamp-GEODE");
    Common::NodeFinderT<osg::Geode> greenFinder("trafficLight::GreenLamp-GEODE");

    if (m_pAmberLamp = amberFinder.find(pAsset)) m_pAmberLamp->ref();
    if (m_pRedLamp = redFinder.find(pAsset)) m_pRedLamp->ref();
    if (m_pGreenLamp = greenFinder.find(pAsset)) m_pGreenLamp->ref();

    // set the current state for the light
    setState(Go);

    // add the selection object
    osg::Geode* pSelectGeode = new osg::Geode();
    osg::ShapeDrawable* pSD = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(0.0f, 0.0f, 0.0f), 150.0f));

    m_pSelectTransform->addChild(pSelectGeode);
    pSelectGeode->addDrawable(pSD);
	m_pTransformation->addChild(m_pSelectTransform);
	m_pSelectTransform->setMatrix(osg::Matrixf::translate(0.0f, 0.0f, 1500.0f));

    // attach the selection handler to allow use to connect the darwable to the facade
	m_pRoot->setUserData(new SelectionHandler(this));

    // the collision switch is currently attached to the asset
    // ideally it should be attached to the matrix transform for this facade
	if(m_pCollisionTargetSwitch)
    {
        m_pTransformation->addChild(m_pCollisionTargetSwitch);
        m_pCollisionTargetSwitch->getParent(0)->removeChild(m_pCollisionTargetSwitch);
    }

    // set the position of the collision target - in this case to the local (0,0,0)
    m_pCollisionTarget->setMatrix(osg::Matrixf::identity());

}

DemoTrafficLight::~DemoTrafficLight()
{
}

osg::Vec3f DemoTrafficLight::getFacadeCollisionPoint()
{
    // currently this is calculating the world position target omn every frame. Ideally, because this is a static object,
    // this position could be calculated in the constructor, stored as a member variable and returned here without repeating the calculation

    osg::Vec3f t, s;
    osg::Quat r, sr;

    // get the path, from the position target to the root, and decompose the resultant matrix to get the world position of the collision target
    osg::computeLocalToWorld(m_pCollisionTarget->getParentalNodePaths(0)[0]).decompose(t, r, s, sr);

    return t;
}

DemoTrafficLight::TLState DemoTrafficLight::getState()
{
    return m_eState;
}

void DemoTrafficLight::setState(TLState eState)
{
    // tis is used to set the state of the lamps and the textures
    m_eState = eState;
    if (m_eState == Go)
    {
        m_pRedLamp->getOrCreateStateSet()->setAttributeAndModes(sm_pGoMaterial, osg::StateAttribute::ON & osg::StateAttribute::OVERRIDE);
        m_pAmberLamp->getOrCreateStateSet()->setAttributeAndModes(sm_pGoMaterial, osg::StateAttribute::ON & osg::StateAttribute::OVERRIDE);
        m_pGreenLamp->getOrCreateStateSet()->setAttributeAndModes(sm_pGoMaterial, osg::StateAttribute::ON & osg::StateAttribute::OVERRIDE);

#ifdef WALK_SHADER
        m_pWalkStateUniform->set(0);
#else
        m_pTextureSwitch->setSingleChildOn(0);
#endif
    }
    else if (m_eState == Stop)
    {
        m_pRedLamp->getOrCreateStateSet()->setAttributeAndModes(sm_pStopMaterial, osg::StateAttribute::ON & osg::StateAttribute::OVERRIDE);
        m_pAmberLamp->getOrCreateStateSet()->setAttributeAndModes(sm_pStopMaterial, osg::StateAttribute::ON & osg::StateAttribute::OVERRIDE);
        m_pGreenLamp->getOrCreateStateSet()->setAttributeAndModes(sm_pStopMaterial, osg::StateAttribute::ON & osg::StateAttribute::OVERRIDE);

#ifdef WALK_SHADER
        m_pWalkStateUniform->set(1);
#else
        m_pTextureSwitch->setSingleChildOn(1);
#endif
    }
}

void DemoTrafficLight::toggleState()
{
    if (m_eState == Go) setState(Stop);
    else setState(Go);

}
#ifdef WALK_SHADER
bool DemoTrafficLight::run(osg::Object* object, osg::Object* data)
{
    //this is the callback function and it is only used for the shader based texture renderer
    if (m_eState==Stop && m_PuiCount <= 100)
    {
        if (m_PuiCount == 100) 
        {
            m_PuiCount = 0;
            toggleState();
        }
        m_PuiCount++;  
    }

    if(osg::NodeVisitor* pNV = dynamic_cast<osg::NodeVisitor*>(data))
    {
        float fFract = (sin((float)pNV->getFrameStamp()->getSimulationTime())+1.0f)/2.0f;
        m_pWalkTimeUniform->set(fFract * 0.7f + 0.3f);
    }

    return false;
}
#endif
