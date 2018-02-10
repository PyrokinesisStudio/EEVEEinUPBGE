/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file RAS_Rasterizer.h
 *  \ingroup bgerast
 */

#ifndef __RAS_RASTERIZER_H__
#define __RAS_RASTERIZER_H__

#ifdef _MSC_VER
#  pragma warning (disable:4786)
#endif

#include "MT_Matrix4x4.h"

#include "RAS_DebugDraw.h"
#include "RAS_Rect.h"

#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <memory>

class RAS_OpenGLRasterizer;
class RAS_ICanvas;
class RAS_IDisplayArray;
class SCA_IScene;
struct KX_ClientObjectInfo;
class KX_RayCast;
struct GPUShader;
struct GPUTexture;
struct GPUViewport;
struct DRWShadingGroup;

/**
 * 3D rendering device context interface. 
 */
class RAS_Rasterizer
{
public:

	/**
	 * Valid SetDepthMask parameters
	 */
	enum DepthMask {
		RAS_DEPTHMASK_ENABLED = 1,
		RAS_DEPTHMASK_DISABLED,
	};

	/**
	 */
	enum {
		RAS_BACKCULL = 16, // GEMAT_BACKCULL
	};

	/**
	 * Mipmap options
	 */
	enum MipmapOption {
		RAS_MIPMAP_NONE,
		RAS_MIPMAP_NEAREST,
		RAS_MIPMAP_LINEAR,

		RAS_MIPMAP_MAX,  /* Should always be last */
	};

	enum EnableBit {
		RAS_DEPTH_TEST = 0,
		RAS_ALPHA_TEST,
		RAS_SCISSOR_TEST,
		RAS_TEXTURE_2D,
		RAS_TEXTURE_CUBE_MAP,
		RAS_BLEND,
		RAS_COLOR_MATERIAL,
		RAS_CULL_FACE,
		RAS_LIGHTING,
		RAS_MULTISAMPLE,
		RAS_POLYGON_STIPPLE,
		RAS_POLYGON_OFFSET_FILL,
		RAS_POLYGON_OFFSET_LINE
	};

	enum DepthFunc {
		RAS_NEVER = 0,
		RAS_LEQUAL,
		RAS_LESS,
		RAS_ALWAYS,
		RAS_GEQUAL,
		RAS_GREATER,
		RAS_NOTEQUAL,
		RAS_EQUAL
	};

	enum BlendFunc {
		RAS_ZERO = 0,
		RAS_ONE,
		RAS_SRC_COLOR,
		RAS_ONE_MINUS_SRC_COLOR,
		RAS_DST_COLOR,
		RAS_ONE_MINUS_DST_COLOR,
		RAS_SRC_ALPHA,
		RAS_ONE_MINUS_SRC_ALPHA,
		RAS_DST_ALPHA,
		RAS_ONE_MINUS_DST_ALPHA,
		RAS_SRC_ALPHA_SATURATE
	};

	enum ClearBit {
		RAS_COLOR_BUFFER_BIT = 0x2,
		RAS_DEPTH_BUFFER_BIT = 0x4,
		RAS_STENCIL_BUFFER_BIT = 0x8
	};

	enum HdrType {
		RAS_HDR_NONE = 0,
		RAS_HDR_HALF_FLOAT,
		RAS_HDR_FULL_FLOAT,
		RAS_HDR_MAX
	};

private:

	// All info used to compute the ray cast transform matrix.
	struct RayCastTranform
	{
		/// The object scale.
		MT_Vector3 scale;
		/// The original object matrix.
		float *origmat;
		/// The output matrix.
		float *mat;
	};

	struct Matrices
	{
		MT_Matrix4x4 view;
		MT_Matrix4x4 viewinv;
		MT_Matrix4x4 proj;
		MT_Matrix4x4 projinv;
		MT_Matrix4x4 pers;
		MT_Matrix4x4 persinv;
	} m_matrices;

	// We store each debug shape by scene.
	std::map<SCA_IScene *, RAS_DebugDraw> m_debugDraws;

	double m_time;
	MT_Matrix4x4 m_viewmatrix;
	MT_Matrix4x4 m_viewinvmatrix;
	MT_Vector3 m_campos;
	bool m_camortho;
	bool m_camnegscale;

	float m_focallength;
	bool m_setfocallength;
	int m_noOfScanlines;

	/* Render tools */
	void *m_clientobject;
	void *m_auxilaryClientInfo;
	int m_lastlightlayer;
	bool m_lastlighting;
	void *m_lastauxinfo;

	bool m_invertFrontFace;
	bool m_last_frontface;

	std::unique_ptr<RAS_OpenGLRasterizer> m_impl;

public:
	RAS_Rasterizer();
	virtual ~RAS_Rasterizer();

	/**
	 * Enable capability
	 * \param bit Enable bit
	 */
	void Enable(EnableBit bit);

	/**
	 * Disable capability
	 * \param bit Enable bit
	 */
	void Disable(EnableBit bit);

	/**
	 * Set the value for Depth Buffer comparisons
	 * \param func Depth comparison function
	 */
	void SetDepthFunc(DepthFunc func);

	/** 
	 * Set the blending equation.
	 * \param src The src value.
	 * \param dst The destination value.
	 */
	void SetBlendFunc(BlendFunc src, BlendFunc dst);

	/**
	 * Takes a screenshot
	 */
	unsigned int *MakeScreenshot(int x, int y, int width, int height);

	/**
	 * SetDepthMask enables or disables writing a fragment's depth value
	 * to the Z buffer.
	 */
	void SetDepthMask(DepthMask depthmask);

	/**
	 * Init initializes the renderer.
	 */
	void Init();

	/**
	 * Exit cleans up the renderer.
	 */
	void Exit();

	/**
	 * BeginFrame is called at the start of each frame.
	 */
	void BeginFrame(double time);

	/**
	 * EndFrame is called at the end of each frame.
	 */
	void EndFrame();

	/**
	 * Clears a specified set of buffers
	 * \param clearbit What buffers to clear (separated by bitwise OR)
	 */
	void Clear(int clearbit);

	/**
	 * Set background color
	 */
	void SetClearColor(float r, float g, float b, float a=1.0f);

	/**
	 * Set background depth
	 */
	void SetClearDepth(float d);

	/**
	 * Set background color mask.
	 */
	void SetColorMask(bool r, bool g, bool b, bool a);

	/**
	 * Draw screen overlay plane with basic uv coordinates.
	 */
	void DrawOverlayPlane();
 
	/// Get the modelview matrix.
	MT_Matrix4x4 GetViewMatrix(const MT_Transform &camtrans, bool perspective);
	/**
	 * Sets the modelview matrix.
	 */
	void SetMatrix(const MT_Matrix4x4 &viewmat, const MT_Matrix4x4& projmat, const MT_Vector3 &pos, const MT_Vector3 &scale);

	/**
	 * Get/Set viewport area
	 */
	void SetViewport(int x, int y, int width, int height);

	/**
	 * Set scissor mask
	 */
	void SetScissor(int x, int y, int width, int height);

	/**
	 */
	const MT_Vector3& GetCameraPosition();
	bool GetCameraOrtho();

	/**
	 * Sets face culling
	 */
	void SetCullFace(bool enable);

	/// Set and enable clip plane.
	void EnableClipPlane(int numplanes);
	/// Disable clip plane
	void DisableClipPlane(int numplanes);

	/**
	 * Sets wireframe mode.
	 */
	void SetLines(bool enable);

	/**
	 */
	double GetTime();

	/**
	 * Generates a projection matrix from the specified frustum.
	 * \param left the left clipping plane
	 * \param right the right clipping plane
	 * \param bottom the bottom clipping plane
	 * \param top the top clipping plane
	 * \param frustnear the near clipping plane
	 * \param frustfar the far clipping plane
	 * \return a 4x4 matrix representing the projection transform.
	 */
	MT_Matrix4x4 GetFrustumMatrix(
	        float left, float right, float bottom, float top,
	        float frustnear, float frustfar,
	        bool perspective = true);

	/**
	 * Generates a orthographic projection matrix from the specified frustum.
	 * \param left the left clipping plane
	 * \param right the right clipping plane
	 * \param bottom the bottom clipping plane
	 * \param top the top clipping plane
	 * \param frustnear the near clipping plane
	 * \param frustfar the far clipping plane
	 * \return a 4x4 matrix representing the projection transform.
	 */
	MT_Matrix4x4 GetOrthoMatrix(
	        float left, float right, float bottom, float top,
	        float frustnear, float frustfar);

	/**
	* GetRenderArea computes the render area from the 2d canvas.
	*/
	RAS_Rect GetRenderArea(RAS_ICanvas *canvas);

	RAS_DebugDraw& GetDebugDraw(SCA_IScene *scene);
	void FlushDebugDraw(SCA_IScene *scene, RAS_ICanvas *canvas);

	const MT_Matrix4x4 &GetViewMatrix() const;
	const MT_Matrix4x4 &GetViewInvMatrix() const;
	const MT_Matrix4x4& GetProjMatrix() const;
	const MT_Matrix4x4& GetProjInvMatrix() const;
	const MT_Matrix4x4& GetPersMatrix() const;
	const MT_Matrix4x4& GetPersInvMatrix() const;

	void SetAlphaBlend(int alphablend);
	void SetFrontFace(bool ccw);

	void SetInvertFrontFace(bool invert);

	void SetAnisotropicFiltering(short level);
	short GetAnisotropicFiltering();

	void SetMipmapping(MipmapOption val);
	MipmapOption GetMipmapping();

	/// \see KX_RayCast
	bool RayHit(KX_ClientObjectInfo *client, KX_RayCast *result, RayCastTranform *raytransform);
	/// \see KX_RayCast
	bool NeedRayCast(KX_ClientObjectInfo *info, void *data);

	/**
	 * Render Tools
	 */
	void GetTransform(float *origmat, int objectdrawmode, float mat[16]);

	void DisableForText();
	/**
	 * Renders 3D text string using BFL.
	 * \param fontid	The id of the font.
	 * \param text		The string to render.
	 * \param size		The size of the text.
	 * \param dpi		The resolution of the text.
	 * \param color		The color of the object.
	 * \param mat		The Matrix of the text object.
	 * \param aspect	A scaling factor to compensate for the size.
	 */
	void RenderText3D(
	        int fontid, const std::string& text, int size, int dpi,
	        const float color[4], const float mat[16], float aspect);

	void SetClientObject(void *obj);

	void SetAuxilaryClientInfo(void *inf);

	/**
	 * Prints information about what the hardware supports.
	 */
	void PrintHardwareInfo();
};

#endif  /* __RAS_RASTERIZER_H__ */
