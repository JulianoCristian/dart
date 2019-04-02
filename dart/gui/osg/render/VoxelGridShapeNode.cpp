/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/osg/render/VoxelGridShapeNode.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>

#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/VoxelGridShape.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class VoxelGridShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  VoxelGridShapeGeode(
      std::shared_ptr<dart::dynamics::VoxelGridShape> shape,
      ShapeFrameNode* parent);

  void refresh();
  void extractData(bool firstTime);

protected:
  virtual ~VoxelGridShapeGeode();

  std::shared_ptr<dart::dynamics::VoxelGridShape> mVoxelGridShape;
  VoxelGridShapeDrawable* mDrawable;

  ::osg::ref_ptr<::osg::LineWidth> mLineWidth;
};

//==============================================================================
class VoxelGridShapeDrawable : public ::osg::Geometry
{
public:
  VoxelGridShapeDrawable(
      dart::dynamics::VoxelGridShape* shape,
      dart::dynamics::VisualAspect* visualAspect);

  void refresh(bool firstTime);

protected:
  virtual ~VoxelGridShapeDrawable();

  dart::dynamics::VoxelGridShape* mVoxelGridShape;
  dart::dynamics::VisualAspect* mVisualAspect;

  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
};

//==============================================================================
VoxelGridShapeNode::VoxelGridShapeNode(
    std::shared_ptr<dart::dynamics::VoxelGridShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mVoxelGridShape(shape), mGeode(nullptr)
{
  mNode = this;
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void VoxelGridShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void VoxelGridShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode)
  {
    mGeode = new VoxelGridShapeGeode(mVoxelGridShape, mParentShapeFrameNode);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
VoxelGridShapeNode::~VoxelGridShapeNode()
{
  // Do nothing
}

//==============================================================================
VoxelGridShapeGeode::VoxelGridShapeGeode(
    std::shared_ptr<dart::dynamics::VoxelGridShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mVoxelGridShape(shape),
    mDrawable(nullptr),
    mLineWidth(new ::osg::LineWidth)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  getOrCreateStateSet()->setMode(GL_LIGHTING, ::osg::StateAttribute::OFF);
  extractData(true);
}

//==============================================================================
void VoxelGridShapeGeode::refresh()
{
  mUtilized = true;

  extractData(false);
}

//==============================================================================
void VoxelGridShapeGeode::extractData(bool firstTime)
{
  //  if (mVoxelGridShape->checkDataVariance(
  //          dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
  //      || firstTime)
  //  {
  //    mLineWidth->setWidth(mVoxelGridShape->getRadius());
  //    getOrCreateStateSet()->setAttributeAndModes(mLineWidth);
  //  }

  //  if (nullptr == mDrawable)
  //  {
  //    mDrawable
  //        = new VoxelGridShapeDrawable(mVoxelGridShape.get(), mVisualAspect);
  //    addDrawable(mDrawable);
  //    return;
  //  }

  //  mDrawable->refresh(false);
}

//==============================================================================
VoxelGridShapeGeode::~VoxelGridShapeGeode()
{
  // Do nothing
}

//==============================================================================
VoxelGridShapeDrawable::VoxelGridShapeDrawable(
    dart::dynamics::VoxelGridShape* shape,
    dart::dynamics::VisualAspect* visualAspect)
  : mVoxelGridShape(shape),
    mVisualAspect(visualAspect),
    mVertices(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array)
{
  refresh(true);
}

//==============================================================================
void VoxelGridShapeDrawable::refresh(bool firstTime)
{
  //  if (mVoxelGridShape->getDataVariance() == dart::dynamics::Shape::STATIC)
  //    setDataVariance(::osg::Object::STATIC);
  //  else
  //    setDataVariance(::osg::Object::DYNAMIC);

  //  const auto& vertices = mVoxelGridShape->getPoints();

  //  if (mVoxelGridShape->checkDataVariance(
  //          dart::dynamics::Shape::DYNAMIC_ELEMENTS)
  //      || firstTime)
  //  {
  //    ::osg::ref_ptr<::osg::DrawElementsUInt> elements
  //        = new ::osg::DrawElementsUInt(GL_POINTS);
  //    elements->reserve(vertices.size());

  //    for (std::size_t i = 0; i < vertices.size(); ++i)
  //      elements->push_back(i);

  //    addPrimitiveSet(elements);
  //  }

  //  if (mVoxelGridShape->checkDataVariance(
  //          dart::dynamics::Shape::DYNAMIC_VERTICES)
  //      || mVoxelGridShape->checkDataVariance(
  //             dart::dynamics::Shape::DYNAMIC_ELEMENTS)
  //      || firstTime)
  //  {
  //    if (mVertices->size() != vertices.size())
  //      mVertices->resize(vertices.size());

  //    for (std::size_t i = 0; i < vertices.size(); ++i)
  //      (*mVertices)[i] = eigToOsgVec3(vertices[i]);

  //    setVertexArray(mVertices);
  //  }

  //  if
  //  (mVoxelGridShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
  //      || firstTime)
  //  {
  //    if (mColors->size() != 1)
  //      mColors->resize(1);

  //    (*mColors)[0] = eigToOsgVec4(mVisualAspect->getRGBA());

  //    setColorArray(mColors, ::osg::Array::BIND_OVERALL);
  //  }
}

//==============================================================================
VoxelGridShapeDrawable::~VoxelGridShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
