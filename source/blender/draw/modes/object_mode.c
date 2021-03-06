/*
 * Copyright 2016, Blender Foundation.
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
 * Contributor(s): Blender Institute
 *
 */

/** \file blender/draw/modes/object_mode.c
 *  \ingroup draw
 */

#include "DRW_engine.h"
#include "DRW_render.h"

#include "DNA_userdef_types.h"
#include "DNA_armature_types.h"
#include "DNA_camera_types.h"
#include "DNA_curve_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meta_types.h"
#include "DNA_object_force_types.h"
#include "DNA_lightprobe_types.h"
#include "DNA_particle_types.h"
#include "DNA_view3d_types.h"
#include "DNA_world_types.h"

#include "BIF_gl.h"

#include "BKE_anim.h"
#include "BKE_camera.h"
#include "BKE_curve.h"
#include "BKE_global.h"
#include "BKE_mball.h"
#include "BKE_object.h"
#include "BKE_particle.h"
#include "BKE_image.h"
#include "BKE_texture.h"

#include "ED_view3d.h"

#include "GPU_shader.h"
#include "GPU_texture.h"

#include "MEM_guardedalloc.h"

#include "UI_resources.h"

#include "draw_mode_engines.h"
#include "draw_manager_text.h"
#include "draw_common.h"

#include "DEG_depsgraph_query.h"

extern struct GPUUniformBuffer *globals_ubo; /* draw_common.c */
extern struct GPUTexture *globals_ramp; /* draw_common.c */
extern GlobalsUboStorage ts;

extern char datatoc_object_outline_resolve_frag_glsl[];
extern char datatoc_object_outline_detect_frag_glsl[];
extern char datatoc_object_outline_expand_frag_glsl[];
extern char datatoc_object_grid_frag_glsl[];
extern char datatoc_object_grid_vert_glsl[];
extern char datatoc_object_empty_image_frag_glsl[];
extern char datatoc_object_empty_image_vert_glsl[];
extern char datatoc_object_lightprobe_grid_vert_glsl[];
extern char datatoc_object_particle_prim_vert_glsl[];
extern char datatoc_object_particle_dot_vert_glsl[];
extern char datatoc_object_particle_dot_frag_glsl[];
extern char datatoc_common_globals_lib_glsl[];
extern char datatoc_common_fxaa_lib_glsl[];
extern char datatoc_gpu_shader_flat_color_frag_glsl[];
extern char datatoc_gpu_shader_fullscreen_vert_glsl[];
extern char datatoc_gpu_shader_uniform_color_frag_glsl[];

/* *********** LISTS *********** */
typedef struct OBJECT_PassList {
	struct DRWPass *non_meshes;
	struct DRWPass *ob_center;
	struct DRWPass *outlines;
	struct DRWPass *outlines_search;
	struct DRWPass *outlines_expand;
	struct DRWPass *outlines_bleed;
	struct DRWPass *outlines_resolve;
	struct DRWPass *grid;
	struct DRWPass *bone_solid;
	struct DRWPass *bone_wire;
	struct DRWPass *bone_envelope;
	struct DRWPass *particle;
	struct DRWPass *lightprobes;
	/* use for empty/background images */
	struct DRWPass *reference_image;
} OBJECT_PassList;

typedef struct OBJECT_FramebufferList {
	struct GPUFrameBuffer *outlines;
	struct GPUFrameBuffer *blur;
} OBJECT_FramebufferList;

typedef struct OBJECT_StorageList {
	struct OBJECT_PrivateData *g_data;
} OBJECT_StorageList;

typedef struct OBJECT_Data {
	void *engine_type;
	OBJECT_FramebufferList *fbl;
	DRWViewportEmptyList *txl;
	OBJECT_PassList *psl;
	OBJECT_StorageList *stl;
} OBJECT_Data;

/* *********** STATIC *********** */

typedef struct OBJECT_PrivateData {
	/* Empties */
	DRWShadingGroup *plain_axes;
	DRWShadingGroup *cube;
	DRWShadingGroup *circle;
	DRWShadingGroup *sphere;
	DRWShadingGroup *cone;
	DRWShadingGroup *single_arrow;
	DRWShadingGroup *single_arrow_line;
	DRWShadingGroup *arrows;
	DRWShadingGroup *axis_names;
	/* GPUTexture -> EmptyImageShadingGroupData */
	GHash *image_plane_map;

	/* Force Field */
	DRWShadingGroup *field_wind;
	DRWShadingGroup *field_force;
	DRWShadingGroup *field_vortex;
	DRWShadingGroup *field_curve_sta;
	DRWShadingGroup *field_curve_end;
	DRWShadingGroup *field_tube_limit;
	DRWShadingGroup *field_cone_limit;

	/* Speaker */
	DRWShadingGroup *speaker;

	/* Probe */
	DRWShadingGroup *probe_cube;
	DRWShadingGroup *probe_planar;
	DRWShadingGroup *probe_grid;

	/* MetaBalls */
	DRWShadingGroup *mball_handle;

	/* Lamps */
	DRWShadingGroup *lamp_center;
	DRWShadingGroup *lamp_center_group;
	DRWShadingGroup *lamp_groundpoint;
	DRWShadingGroup *lamp_groundline;
	DRWShadingGroup *lamp_circle;
	DRWShadingGroup *lamp_circle_shadow;
	DRWShadingGroup *lamp_sunrays;
	DRWShadingGroup *lamp_distance;
	DRWShadingGroup *lamp_buflimit;
	DRWShadingGroup *lamp_buflimit_points;
	DRWShadingGroup *lamp_area;
	DRWShadingGroup *lamp_hemi;
	DRWShadingGroup *lamp_spot_cone;
	DRWShadingGroup *lamp_spot_blend;
	DRWShadingGroup *lamp_spot_pyramid;
	DRWShadingGroup *lamp_spot_blend_rect;

	/* Helpers */
	DRWShadingGroup *relationship_lines;

	/* Objects Centers */
	DRWShadingGroup *center_active;
	DRWShadingGroup *center_selected;
	DRWShadingGroup *center_deselected;
	DRWShadingGroup *center_selected_lib;
	DRWShadingGroup *center_deselected_lib;

	/* Camera */
	DRWShadingGroup *camera;
	DRWShadingGroup *camera_frame;
	DRWShadingGroup *camera_tria;
	DRWShadingGroup *camera_focus;
	DRWShadingGroup *camera_clip;
	DRWShadingGroup *camera_clip_points;
	DRWShadingGroup *camera_mist;
	DRWShadingGroup *camera_mist_points;

	/* Outlines */
	DRWShadingGroup *outlines_active;
	DRWShadingGroup *outlines_active_group;
	DRWShadingGroup *outlines_select;
	DRWShadingGroup *outlines_select_group;
	DRWShadingGroup *outlines_transform;

	/* Lightprobes */
	DRWShadingGroup *lightprobes_cube;
	DRWShadingGroup *lightprobes_planar;

	/* Wire */
	DRWShadingGroup *wire;
	DRWShadingGroup *wire_active;
	DRWShadingGroup *wire_active_group;
	DRWShadingGroup *wire_select;
	DRWShadingGroup *wire_select_group;
	DRWShadingGroup *wire_transform;
} OBJECT_PrivateData; /* Transient data */

static struct {
	/* Instance Data format */
	struct Gwn_VertFormat *empty_image_format;
	struct Gwn_VertFormat *empty_image_wire_format;

	/* fullscreen shaders */
	GPUShader *outline_resolve_sh;
	GPUShader *outline_resolve_aa_sh;
	GPUShader *outline_detect_sh;
	GPUShader *outline_fade_sh;

	/* regular shaders */
	GPUShader *object_empty_image_sh;
	GPUShader *object_empty_image_wire_sh;
	GPUShader *grid_sh;
	GPUShader *part_dot_sh;
	GPUShader *part_prim_sh;
	GPUShader *part_axis_sh;
	GPUShader *lightprobe_grid_sh;
	float camera_pos[3];
	float screenvecs[3][4];
	float grid_settings[5];
	int grid_flag;
	float grid_normal[3];
	float grid_axes[3];
	int zpos_flag;
	int zneg_flag;
	float zplane_normal[3];
	float zplane_axes[3];
	float inv_viewport_size[2];
	bool draw_grid;
	/* Temp buffer textures */
	struct GPUTexture *outlines_depth_tx;
	struct GPUTexture *outlines_color_tx;
	struct GPUTexture *outlines_blur_tx;
} e_data = {NULL}; /* Engine data */


enum {
	SHOW_AXIS_X  = (1 << 0),
	SHOW_AXIS_Y  = (1 << 1),
	SHOW_AXIS_Z  = (1 << 2),
	SHOW_GRID    = (1 << 3),
	PLANE_XY     = (1 << 4),
	PLANE_XZ     = (1 << 5),
	PLANE_YZ     = (1 << 6),
	CLIP_ZPOS    = (1 << 7),
	CLIP_ZNEG    = (1 << 8),
};

/* *********** FUNCTIONS *********** */

static void OBJECT_engine_init(void *vedata)
{
	OBJECT_FramebufferList *fbl = ((OBJECT_Data *)vedata)->fbl;

	const float *viewport_size = DRW_viewport_size_get();

	if (DRW_state_is_fbo()) {
		DRWFboTexture tex[2] = {
			{&e_data.outlines_depth_tx, DRW_TEX_DEPTH_24, DRW_TEX_TEMP},
			{&e_data.outlines_color_tx, DRW_TEX_RGBA_8, DRW_TEX_FILTER | DRW_TEX_TEMP},
		};

		DRW_framebuffer_init(
		        &fbl->outlines, &draw_engine_object_type,
		        (int)viewport_size[0], (int)viewport_size[1],
		        tex, 2);

		DRWFboTexture blur_tex = {&e_data.outlines_blur_tx, DRW_TEX_RGBA_8, DRW_TEX_FILTER | DRW_TEX_TEMP};
		DRW_framebuffer_init(
		        &fbl->blur, &draw_engine_object_type,
		        (int)viewport_size[0], (int)viewport_size[1],
		        &blur_tex, 1);
	}

	if (!e_data.outline_resolve_sh) {
		e_data.outline_resolve_sh = DRW_shader_create_fullscreen(datatoc_object_outline_resolve_frag_glsl, NULL);
	}

	if (!e_data.outline_resolve_aa_sh) {
		e_data.outline_resolve_aa_sh = DRW_shader_create_with_lib(
		            datatoc_gpu_shader_fullscreen_vert_glsl, NULL,
		            datatoc_object_outline_resolve_frag_glsl,
		            datatoc_common_fxaa_lib_glsl,
		            "#define FXAA_ALPHA\n"
		            "#define USE_FXAA\n");
	}

	if (!e_data.outline_detect_sh) {
		e_data.outline_detect_sh = DRW_shader_create_fullscreen(datatoc_object_outline_detect_frag_glsl, NULL);
	}

	if (!e_data.outline_fade_sh) {
		e_data.outline_fade_sh = DRW_shader_create_fullscreen(datatoc_object_outline_expand_frag_glsl, NULL);
	}

	if (!e_data.object_empty_image_sh) {
		e_data.object_empty_image_sh = DRW_shader_create_with_lib(
		           datatoc_object_empty_image_vert_glsl, NULL,
		           datatoc_object_empty_image_frag_glsl,
		           datatoc_common_globals_lib_glsl, NULL);
	}

	if (!e_data.object_empty_image_wire_sh) {
		e_data.object_empty_image_wire_sh = DRW_shader_create_with_lib(
		           datatoc_object_empty_image_vert_glsl, NULL,
		           datatoc_object_empty_image_frag_glsl,
		           datatoc_common_globals_lib_glsl,
		           "#define USE_WIRE\n");
	}

	if (!e_data.grid_sh) {
		e_data.grid_sh = DRW_shader_create_with_lib(
		        datatoc_object_grid_vert_glsl, NULL,
		        datatoc_object_grid_frag_glsl,
		        datatoc_common_globals_lib_glsl, NULL);
	}

	if (!e_data.part_prim_sh) {
		e_data.part_prim_sh = DRW_shader_create(
		        datatoc_object_particle_prim_vert_glsl, NULL, datatoc_gpu_shader_flat_color_frag_glsl, NULL);
	}

	if (!e_data.part_axis_sh) {
		e_data.part_axis_sh = DRW_shader_create(
		        datatoc_object_particle_prim_vert_glsl, NULL, datatoc_gpu_shader_flat_color_frag_glsl,
		        "#define USE_AXIS\n");
	}

	if (!e_data.part_dot_sh) {
		e_data.part_dot_sh = DRW_shader_create(
		        datatoc_object_particle_dot_vert_glsl, NULL, datatoc_object_particle_dot_frag_glsl, NULL);
	}

	if (!e_data.lightprobe_grid_sh) {
		e_data.lightprobe_grid_sh = DRW_shader_create(
		        datatoc_object_lightprobe_grid_vert_glsl, NULL, datatoc_gpu_shader_uniform_color_frag_glsl, NULL);
	}

	{
		/* Grid precompute */
		float invviewmat[4][4], invwinmat[4][4];
		float viewmat[4][4], winmat[4][4];
		const DRWContextState *draw_ctx = DRW_context_state_get();
		View3D *v3d = draw_ctx->v3d;
		Scene *scene = draw_ctx->scene;
		RegionView3D *rv3d = draw_ctx->rv3d;
		float grid_scale = ED_view3d_grid_scale(scene, v3d, NULL);
		float grid_res;

		const bool show_axis_x = (v3d->gridflag & V3D_SHOW_X) != 0;
		const bool show_axis_y = (v3d->gridflag & V3D_SHOW_Y) != 0;
		const bool show_axis_z = (v3d->gridflag & V3D_SHOW_Z) != 0;
		const bool show_floor = (v3d->gridflag & V3D_SHOW_FLOOR) != 0;
		e_data.draw_grid = show_axis_x || show_axis_y || show_axis_z || show_floor;

		DRW_viewport_matrix_get(winmat, DRW_MAT_WIN);
		DRW_viewport_matrix_get(viewmat, DRW_MAT_VIEW);
		DRW_viewport_matrix_get(invwinmat, DRW_MAT_WININV);
		DRW_viewport_matrix_get(invviewmat, DRW_MAT_VIEWINV);

		/* Setup camera pos */
		copy_v3_v3(e_data.camera_pos, invviewmat[3]);

		/* if perps */
		if (winmat[3][3] == 0.0f) {
			float fov;
			float viewvecs[2][4] = {
			    {1.0f, -1.0f, -1.0f, 1.0f},
			    {-1.0f, 1.0f, -1.0f, 1.0f}
			};

			/* convert the view vectors to view space */
			for (int i = 0; i < 2; i++) {
				mul_m4_v4(invwinmat, viewvecs[i]);
				mul_v3_fl(viewvecs[i], 1.0f / viewvecs[i][2]); /* perspective divide */
			}

			fov = angle_v3v3(viewvecs[0], viewvecs[1]) / 2.0f;
			grid_res = fabsf(tanf(fov)) / grid_scale;

			e_data.grid_flag = (1 << 4); /* XY plane */
			if (show_axis_x)
				e_data.grid_flag |= SHOW_AXIS_X;
			if (show_axis_y)
				e_data.grid_flag |= SHOW_AXIS_Y;
			if (show_floor)
				e_data.grid_flag |= SHOW_GRID;

		}
		else {
			float viewdist = 1.0f / max_ff(fabsf(winmat[0][0]), fabsf(winmat[1][1]));
			grid_res = viewdist / grid_scale;

			if (ELEM(rv3d->view, RV3D_VIEW_RIGHT, RV3D_VIEW_LEFT)) {
				e_data.grid_flag = PLANE_YZ;
				e_data.grid_flag |= SHOW_AXIS_Y;
				e_data.grid_flag |= SHOW_AXIS_Z;
				e_data.grid_flag |= SHOW_GRID;
			}
			else if (ELEM(rv3d->view, RV3D_VIEW_TOP, RV3D_VIEW_BOTTOM)) {
				e_data.grid_flag = PLANE_XY;
				e_data.grid_flag |= SHOW_AXIS_X;
				e_data.grid_flag |= SHOW_AXIS_Y;
				e_data.grid_flag |= SHOW_GRID;
			}
			else if (ELEM(rv3d->view, RV3D_VIEW_FRONT, RV3D_VIEW_BACK)) {
				e_data.grid_flag = PLANE_XZ;
				e_data.grid_flag |= SHOW_AXIS_X;
				e_data.grid_flag |= SHOW_AXIS_Z;
				e_data.grid_flag |= SHOW_GRID;
			}
			else { /* RV3D_VIEW_USER */
				e_data.grid_flag = PLANE_XY;
				if (show_axis_x)
					e_data.grid_flag |= SHOW_AXIS_X;
				if (show_axis_y)
					e_data.grid_flag |= SHOW_AXIS_Y;
				if (show_floor)
					e_data.grid_flag |= SHOW_GRID;
			}
		}

		e_data.grid_normal[0] = (float)((e_data.grid_flag & PLANE_YZ) != 0);
		e_data.grid_normal[1] = (float)((e_data.grid_flag & PLANE_XZ) != 0);
		e_data.grid_normal[2] = (float)((e_data.grid_flag & PLANE_XY) != 0);

		e_data.grid_axes[0] = (float)((e_data.grid_flag & (PLANE_XZ | PLANE_XY)) != 0);
		e_data.grid_axes[1] = (float)((e_data.grid_flag & (PLANE_YZ | PLANE_XY)) != 0);
		e_data.grid_axes[2] = (float)((e_data.grid_flag & (PLANE_YZ | PLANE_XZ)) != 0);

		/* Vectors to recover pixel world position. Fix grid precision issue. */
		/* Using pixel at z = 0.0f in ndc space : gives average precision between
		 * near and far plane. Note that it might not be the best choice. */
		copy_v4_fl4(e_data.screenvecs[0],  1.0f, -1.0f, 0.0f, 1.0f);
		copy_v4_fl4(e_data.screenvecs[1], -1.0f,  1.0f, 0.0f, 1.0f);
		copy_v4_fl4(e_data.screenvecs[2], -1.0f, -1.0f, 0.0f, 1.0f);

		for (int i = 0; i < 3; i++) {
			/* Doing 2 steps to recover world position of the corners of the frustum.
			 * Using the inverse perspective matrix is giving very low precision output. */
			mul_m4_v4(invwinmat, e_data.screenvecs[i]);
			e_data.screenvecs[i][0] /= e_data.screenvecs[i][3]; /* perspective divide */
			e_data.screenvecs[i][1] /= e_data.screenvecs[i][3]; /* perspective divide */
			e_data.screenvecs[i][2] /= e_data.screenvecs[i][3]; /* perspective divide */
			e_data.screenvecs[i][3] = 1.0f;
			/* main instability come from this one */
			/* TODO : to make things even more stable, don't use
			 * invviewmat and derive vectors from camera properties */
			mul_m4_v4(invviewmat, e_data.screenvecs[i]);
		}

		sub_v3_v3(e_data.screenvecs[0], e_data.screenvecs[2]);
		sub_v3_v3(e_data.screenvecs[1], e_data.screenvecs[2]);

		/* Z axis if needed */
		if (((rv3d->view == RV3D_VIEW_USER) || (rv3d->persp != RV3D_ORTHO)) && show_axis_z) {
			e_data.zpos_flag = SHOW_AXIS_Z;

			float zvec[4] = {0.0f, 0.0f, -1.0f, 0.0f};
			mul_m4_v4(invviewmat, zvec);

			/* z axis : chose the most facing plane */
			if (fabsf(zvec[0]) < fabsf(zvec[1])) {
				e_data.zpos_flag |= PLANE_XZ;
			}
			else {
				e_data.zpos_flag |= PLANE_YZ;
			}

			e_data.zneg_flag = e_data.zpos_flag;

			/* Persp : If camera is below floor plane, we switch clipping
			 * Ortho : If eye vector is looking up, we switch clipping */
			if (((winmat[3][3] == 0.0f) && (e_data.camera_pos[2] > 0.0f)) ||
			    ((winmat[3][3] != 0.0f) && (zvec[2] < 0.0f)))
			{
				e_data.zpos_flag |= CLIP_ZPOS;
				e_data.zneg_flag |= CLIP_ZNEG;
			}
			else {
				e_data.zpos_flag |= CLIP_ZNEG;
				e_data.zneg_flag |= CLIP_ZPOS;
			}

			e_data.zplane_normal[0] = (float)((e_data.zpos_flag & PLANE_YZ) != 0);
			e_data.zplane_normal[1] = (float)((e_data.zpos_flag & PLANE_XZ) != 0);
			e_data.zplane_normal[2] = (float)((e_data.zpos_flag & PLANE_XY) != 0);

			e_data.zplane_axes[0] = (float)((e_data.zpos_flag & (PLANE_XZ | PLANE_XY)) != 0);
			e_data.zplane_axes[1] = (float)((e_data.zpos_flag & (PLANE_YZ | PLANE_XY)) != 0);
			e_data.zplane_axes[2] = (float)((e_data.zpos_flag & (PLANE_YZ | PLANE_XZ)) != 0);

		}
		else {
			e_data.zneg_flag = e_data.zpos_flag = CLIP_ZNEG | CLIP_ZPOS;
		}

		float dist;
		if (rv3d->persp == RV3D_CAMOB && v3d->camera) {
			Object *camera_object = DEG_get_evaluated_object(draw_ctx->depsgraph, v3d->camera);
			dist = ((Camera *)camera_object)->clipend;
		}
		else {
			dist = v3d->far;
		}

		e_data.grid_settings[0] = dist / 2.0f; /* gridDistance */
		e_data.grid_settings[1] = grid_res; /* gridResolution */
		e_data.grid_settings[2] = grid_scale; /* gridScale */
		e_data.grid_settings[3] = v3d->gridsubdiv; /* gridSubdiv */
		e_data.grid_settings[4] = (v3d->gridsubdiv > 1) ? 1.0f / logf(v3d->gridsubdiv) : 0.0f; /* 1/log(gridSubdiv) */
	}

	copy_v2_v2(e_data.inv_viewport_size, DRW_viewport_size_get());
	invert_v2(e_data.inv_viewport_size);
}

static void OBJECT_engine_free(void)
{
	MEM_SAFE_FREE(e_data.empty_image_format);
	MEM_SAFE_FREE(e_data.empty_image_wire_format);
	DRW_SHADER_FREE_SAFE(e_data.outline_resolve_sh);
	DRW_SHADER_FREE_SAFE(e_data.outline_resolve_aa_sh);
	DRW_SHADER_FREE_SAFE(e_data.outline_detect_sh);
	DRW_SHADER_FREE_SAFE(e_data.outline_fade_sh);
	DRW_SHADER_FREE_SAFE(e_data.object_empty_image_sh);
	DRW_SHADER_FREE_SAFE(e_data.object_empty_image_wire_sh);
	DRW_SHADER_FREE_SAFE(e_data.grid_sh);
	DRW_SHADER_FREE_SAFE(e_data.part_prim_sh);
	DRW_SHADER_FREE_SAFE(e_data.part_axis_sh);
	DRW_SHADER_FREE_SAFE(e_data.part_dot_sh);
	DRW_SHADER_FREE_SAFE(e_data.lightprobe_grid_sh);
}

static DRWShadingGroup *shgroup_outline(DRWPass *pass, const float col[4], GPUShader *sh)
{
	DRWShadingGroup *grp = DRW_shgroup_create(sh, pass);
	DRW_shgroup_uniform_vec4(grp, "color", col, 1);

	return grp;
}

/* currently same as 'shgroup_outline', new function to avoid confustion */
static DRWShadingGroup *shgroup_wire(DRWPass *pass, const float col[4], GPUShader *sh)
{
	DRWShadingGroup *grp = DRW_shgroup_create(sh, pass);
	DRW_shgroup_uniform_vec4(grp, "color", col, 1);

	return grp;
}

static DRWShadingGroup *shgroup_theme_id_to_outline_or(
        OBJECT_StorageList *stl, int theme_id, DRWShadingGroup *fallback)
{
	switch (theme_id) {
		case TH_ACTIVE:
			return stl->g_data->outlines_active;
		case TH_SELECT:
			return stl->g_data->outlines_select;
		case TH_GROUP_ACTIVE:
			return stl->g_data->outlines_select_group;
		case TH_TRANSFORM:
			return stl->g_data->outlines_transform;
		default:
			return fallback;
	}
}

static DRWShadingGroup *shgroup_theme_id_to_wire_or(
        OBJECT_StorageList *stl, int theme_id, DRWShadingGroup *fallback)
{
	switch (theme_id) {
		case TH_ACTIVE:
			return stl->g_data->wire_active;
		case TH_SELECT:
			return stl->g_data->wire_select;
		case TH_GROUP_ACTIVE:
			return stl->g_data->wire_select_group;
		case TH_TRANSFORM:
			return stl->g_data->wire_transform;
		default:
			return fallback;
	}
}

static void image_calc_aspect(Image *ima, ImageUser *iuser, float r_image_aspect[2])
{
	float ima_x, ima_y;
	if (ima) {
		int w, h;
		BKE_image_get_size(ima, iuser, &w, &h);
		ima_x = w;
		ima_y = h;
	}
	else {
		/* if no image, make it a 1x1 empty square, honor scale & offset */
		ima_x = ima_y = 1.0f;
	}
	/* Get the image aspect even if the buffer is invalid */
	float sca_x = 1.0f, sca_y = 1.0f;
	if (ima) {
		if (ima->aspx > ima->aspy) {
			sca_y = ima->aspy / ima->aspx;
		}
		else if (ima->aspx < ima->aspy) {
			sca_x = ima->aspx / ima->aspy;
		}
	}

	const float scale_x_inv = ima_x * sca_x;
	const float scale_y_inv = ima_y * sca_y;
	if (scale_x_inv > scale_y_inv) {
		r_image_aspect[0] = 1.0f;
		r_image_aspect[1] = scale_y_inv / scale_x_inv;
	}
	else {
		r_image_aspect[0] = scale_x_inv / scale_y_inv;
		r_image_aspect[1] = 1.0f;
	}
}

/* per-image shading groups for image-type empty objects */
struct EmptyImageShadingGroupData {
	DRWShadingGroup *shgrp_image;
	DRWShadingGroup *shgrp_wire;
	float image_aspect[2];
};

static void DRW_shgroup_empty_image(
        OBJECT_StorageList *stl, OBJECT_PassList *psl, Object *ob, const float color[3])
{
	/* TODO: 'StereoViews', see draw_empty_image. */

	if (stl->g_data->image_plane_map == NULL) {
		stl->g_data->image_plane_map = BLI_ghash_ptr_new(__func__);
	}

	struct EmptyImageShadingGroupData *empty_image_data;

	GPUTexture *tex = ob->data ?
	        GPU_texture_from_blender(ob->data, ob->iuser, GL_TEXTURE_2D, false, false, false) : NULL;
	void **val_p;

	/* Create on demand, 'tex' may be NULL. */
	if (BLI_ghash_ensure_p(stl->g_data->image_plane_map, tex, &val_p)) {
		empty_image_data = *val_p;
	}
	else {
		empty_image_data = MEM_mallocN(sizeof(*empty_image_data), __func__);

		image_calc_aspect(ob->data, ob->iuser, empty_image_data->image_aspect);

		if (tex) {
			DRW_shgroup_instance_format(e_data.empty_image_format, {
				{"objectColor"         , DRW_ATTRIB_FLOAT, 4},
				{"size"                , DRW_ATTRIB_FLOAT, 1},
				{"offset"              , DRW_ATTRIB_FLOAT, 2},
				{"InstanceModelMatrix" , DRW_ATTRIB_FLOAT, 16},
			});

			struct Gwn_Batch *geom = DRW_cache_image_plane_get();
			DRWShadingGroup *grp = DRW_shgroup_instance_create(
			        e_data.object_empty_image_sh, psl->non_meshes, geom, e_data.empty_image_format);
			DRW_shgroup_uniform_texture(grp, "image", tex);
			DRW_shgroup_uniform_vec2(grp, "aspect", empty_image_data->image_aspect, 1);

			empty_image_data->shgrp_image = grp;
		}
		else {
			empty_image_data->shgrp_image = NULL;
		}

		{
			DRW_shgroup_instance_format(e_data.empty_image_wire_format, {
				{"objectColor"         , DRW_ATTRIB_FLOAT, 4},
				{"size"                , DRW_ATTRIB_FLOAT, 1},
				{"offset"              , DRW_ATTRIB_FLOAT, 2},
				{"InstanceModelMatrix" , DRW_ATTRIB_FLOAT, 16}
			});

			struct Gwn_Batch *geom = DRW_cache_image_plane_wire_get();
			DRWShadingGroup *grp = DRW_shgroup_instance_create(
			        e_data.object_empty_image_wire_sh, psl->non_meshes, geom, e_data.empty_image_wire_format);
			DRW_shgroup_uniform_vec2(grp, "aspect", empty_image_data->image_aspect, 1);

			empty_image_data->shgrp_wire = grp;
		}

		*val_p = empty_image_data;
	}

	if (empty_image_data->shgrp_image != NULL) {
		DRW_shgroup_call_dynamic_add(
		        empty_image_data->shgrp_image,
		        ob->col,
		        &ob->empty_drawsize,
		        ob->ima_ofs,
		        ob->obmat);
	}

	DRW_shgroup_call_dynamic_add(
	        empty_image_data->shgrp_wire,
	        color,
	        &ob->empty_drawsize,
	        ob->ima_ofs,
	        ob->obmat);
}

static void OBJECT_cache_init(void *vedata)
{
	OBJECT_PassList *psl = ((OBJECT_Data *)vedata)->psl;
	OBJECT_StorageList *stl = ((OBJECT_Data *)vedata)->stl;
	DefaultTextureList *dtxl = DRW_viewport_texture_list_get();

	if (!stl->g_data) {
		/* Alloc transient pointers */
		stl->g_data = MEM_mallocN(sizeof(*stl->g_data), __func__);
	}

	{
		DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS | DRW_STATE_WIRE;
		psl->outlines = DRW_pass_create("Outlines Depth Pass", state);

		GPUShader *sh = GPU_shader_get_builtin_shader(GPU_SHADER_3D_UNIFORM_COLOR);

		/* Select */
		stl->g_data->outlines_select = shgroup_outline(psl->outlines, ts.colorSelect, sh);
		stl->g_data->outlines_select_group = shgroup_outline(psl->outlines, ts.colorGroupActive, sh);

		/* Transform */
		stl->g_data->outlines_transform = shgroup_outline(psl->outlines, ts.colorTransform, sh);

		/* Active */
		stl->g_data->outlines_active = shgroup_outline(psl->outlines, ts.colorActive, sh);
		stl->g_data->outlines_active_group = shgroup_outline(psl->outlines, ts.colorGroupActive, sh);
	}

	{
		DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS;
		psl->lightprobes = DRW_pass_create("Object Probe Pass", state);

		/* Cubemap */
		stl->g_data->lightprobes_cube = shgroup_instance(psl->lightprobes, DRW_cache_sphere_get());

		/* Planar */
		stl->g_data->lightprobes_planar = shgroup_instance(psl->lightprobes, DRW_cache_quad_get());
	}

	{
		DRWState state = DRW_STATE_WRITE_COLOR;
		struct Gwn_Batch *quad = DRW_cache_fullscreen_quad_get();
		static float alphaOcclu = 0.35f;
		/* Reminder : bool uniforms need to be 4 bytes. */
		static const int bTrue = true;
		static const int bFalse = false;

		psl->outlines_search = DRW_pass_create("Outlines Detect Pass", state);

		DRWShadingGroup *grp = DRW_shgroup_create(e_data.outline_detect_sh, psl->outlines_search);
		DRW_shgroup_uniform_buffer(grp, "outlineColor", &e_data.outlines_color_tx);
		DRW_shgroup_uniform_buffer(grp, "outlineDepth", &e_data.outlines_depth_tx);
		DRW_shgroup_uniform_buffer(grp, "sceneDepth", &dtxl->depth);
		DRW_shgroup_uniform_float(grp, "alphaOcclu", &alphaOcclu, 1);
		DRW_shgroup_call_add(grp, quad, NULL);

		psl->outlines_expand = DRW_pass_create("Outlines Expand Pass", state);

		grp = DRW_shgroup_create(e_data.outline_fade_sh, psl->outlines_expand);
		DRW_shgroup_uniform_buffer(grp, "outlineColor", &e_data.outlines_blur_tx);
		DRW_shgroup_uniform_bool(grp, "doExpand", &bTrue, 1);
		DRW_shgroup_call_add(grp, quad, NULL);

		psl->outlines_bleed = DRW_pass_create("Outlines Bleed Pass", state);

		grp = DRW_shgroup_create(e_data.outline_fade_sh, psl->outlines_bleed);
		DRW_shgroup_uniform_buffer(grp, "outlineColor", &e_data.outlines_color_tx);
		DRW_shgroup_uniform_bool(grp, "doExpand", &bFalse, 1);
		DRW_shgroup_call_add(grp, quad, NULL);
	}

	{
		DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND;
		psl->outlines_resolve = DRW_pass_create("Outlines Resolve Pass", state);

		struct Gwn_Batch *quad = DRW_cache_fullscreen_quad_get();

		DRWShadingGroup *grp = DRW_shgroup_create(e_data.outline_resolve_aa_sh, psl->outlines_resolve);
		DRW_shgroup_uniform_buffer(grp, "outlineBluredColor", &e_data.outlines_blur_tx);
		DRW_shgroup_uniform_vec2(grp, "rcpDimensions", e_data.inv_viewport_size, 1);
		DRW_shgroup_call_add(grp, quad, NULL);
	}

	{
		/* Grid pass */
		DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND;
		psl->grid = DRW_pass_create("Infinite Grid Pass", state);

		struct Gwn_Batch *quad = DRW_cache_fullscreen_quad_get();
		static float mat[4][4];
		unit_m4(mat);

		/* Create 3 quads to render ordered transparency Z axis */
		DRWShadingGroup *grp = DRW_shgroup_create(e_data.grid_sh, psl->grid);
		DRW_shgroup_uniform_int(grp, "gridFlag", &e_data.zneg_flag, 1);
		DRW_shgroup_uniform_vec3(grp, "planeNormal", e_data.zplane_normal, 1);
		DRW_shgroup_uniform_vec3(grp, "planeAxes", e_data.zplane_axes, 1);
		DRW_shgroup_uniform_vec3(grp, "cameraPos", e_data.camera_pos, 1);
		DRW_shgroup_uniform_vec4(grp, "screenvecs[0]", e_data.screenvecs[0], 3);
		DRW_shgroup_uniform_vec4(grp, "gridSettings", e_data.grid_settings, 1);
		DRW_shgroup_uniform_float(grp, "gridOneOverLogSubdiv", &e_data.grid_settings[4], 1);
		DRW_shgroup_uniform_block(grp, "globalsBlock", globals_ubo);
		DRW_shgroup_uniform_vec2(grp, "viewportSize", DRW_viewport_size_get(), 1);
		DRW_shgroup_uniform_buffer(grp, "depthBuffer", &dtxl->depth);
		DRW_shgroup_call_add(grp, quad, mat);

		grp = DRW_shgroup_create(e_data.grid_sh, psl->grid);
		DRW_shgroup_uniform_int(grp, "gridFlag", &e_data.grid_flag, 1);
		DRW_shgroup_uniform_vec3(grp, "planeNormal", e_data.grid_normal, 1);
		DRW_shgroup_uniform_vec3(grp, "planeAxes", e_data.grid_axes, 1);
		DRW_shgroup_uniform_block(grp, "globalsBlock", globals_ubo);
		DRW_shgroup_uniform_buffer(grp, "depthBuffer", &dtxl->depth);
		DRW_shgroup_call_add(grp, quad, mat);

		grp = DRW_shgroup_create(e_data.grid_sh, psl->grid);
		DRW_shgroup_uniform_int(grp, "gridFlag", &e_data.zpos_flag, 1);
		DRW_shgroup_uniform_vec3(grp, "planeNormal", e_data.zplane_normal, 1);
		DRW_shgroup_uniform_vec3(grp, "planeAxes", e_data.zplane_axes, 1);
		DRW_shgroup_uniform_block(grp, "globalsBlock", globals_ubo);
		DRW_shgroup_uniform_buffer(grp, "depthBuffer", &dtxl->depth);
		DRW_shgroup_call_add(grp, quad, mat);
	}

	{
		/* Solid bones */
		DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS;
		psl->bone_solid = DRW_pass_create("Bone Solid Pass", state);
	}

	{
		/* Wire bones */
		DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS | DRW_STATE_BLEND;
		psl->bone_wire = DRW_pass_create("Bone Wire Pass", state);
	}

	{
		/* distance outline around envelope bones */
		DRWState state = DRW_STATE_ADDITIVE | DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS | DRW_STATE_BLEND;
		psl->bone_envelope = DRW_pass_create("Bone Envelope Outline Pass", state);
	}

	{
		/* Non Meshes Pass (Camera, empties, lamps ...) */
		struct Gwn_Batch *geom;

		DRWState state =
		        DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
		        DRW_STATE_DEPTH_LESS | DRW_STATE_BLEND | DRW_STATE_POINT;
		state |= DRW_STATE_WIRE;
		psl->non_meshes = DRW_pass_create("Non Meshes Pass", state);

		/* Empties */
		geom = DRW_cache_plain_axes_get();
		stl->g_data->plain_axes = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_cube_get();
		stl->g_data->cube = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_circle_get();
		stl->g_data->circle = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_empty_sphere_get();
		stl->g_data->sphere = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_empty_cone_get();
		stl->g_data->cone = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_single_arrow_get();
		stl->g_data->single_arrow = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_single_line_get();
		stl->g_data->single_arrow_line = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_arrows_get();
		stl->g_data->arrows = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_axis_names_get();
		stl->g_data->axis_names = shgroup_instance_axis_names(psl->non_meshes, geom);

		/* initialize on first use */
		stl->g_data->image_plane_map = NULL;

		/* Force Field */
		geom = DRW_cache_field_wind_get();
		stl->g_data->field_wind = shgroup_instance_scaled(psl->non_meshes, geom);

		geom = DRW_cache_field_force_get();
		stl->g_data->field_force = shgroup_instance_screen_aligned(psl->non_meshes, geom);

		geom = DRW_cache_field_vortex_get();
		stl->g_data->field_vortex = shgroup_instance_scaled(psl->non_meshes, geom);

		geom = DRW_cache_screenspace_circle_get();
		stl->g_data->field_curve_sta = shgroup_instance_screen_aligned(psl->non_meshes, geom);

		/* Speaker */
		geom = DRW_cache_speaker_get();
		stl->g_data->speaker = shgroup_instance(psl->non_meshes, geom);

		/* Probe */
		static float probeSize = 14.0f;
		geom = DRW_cache_lightprobe_cube_get();
		stl->g_data->probe_cube = shgroup_instance_screenspace(psl->non_meshes, geom, &probeSize);

		geom = DRW_cache_lightprobe_grid_get();
		stl->g_data->probe_grid = shgroup_instance_screenspace(psl->non_meshes, geom, &probeSize);

		static float probePlanarSize = 20.0f;
		geom = DRW_cache_lightprobe_planar_get();
		stl->g_data->probe_planar = shgroup_instance_screenspace(psl->non_meshes, geom, &probePlanarSize);

		/* Camera */
		geom = DRW_cache_camera_get();
		stl->g_data->camera = shgroup_camera_instance(psl->non_meshes, geom);

		geom = DRW_cache_camera_frame_get();
		stl->g_data->camera_frame = shgroup_camera_instance(psl->non_meshes, geom);

		geom = DRW_cache_camera_tria_get();
		stl->g_data->camera_tria = shgroup_camera_instance(psl->non_meshes, geom);

		geom = DRW_cache_plain_axes_get();
		stl->g_data->camera_focus = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_single_line_get();
		stl->g_data->camera_clip = shgroup_distance_lines_instance(psl->non_meshes, geom);
		stl->g_data->camera_mist = shgroup_distance_lines_instance(psl->non_meshes, geom);

		geom = DRW_cache_single_line_endpoints_get();
		stl->g_data->camera_clip_points = shgroup_distance_lines_instance(psl->non_meshes, geom);
		stl->g_data->camera_mist_points = shgroup_distance_lines_instance(psl->non_meshes, geom);

	}

	{
		GPUShader *sh = GPU_shader_get_builtin_shader(GPU_SHADER_3D_UNIFORM_COLOR);

		/* Unselected */
		stl->g_data->wire = shgroup_wire(psl->non_meshes, ts.colorWire, sh);

		/* Select */
		stl->g_data->wire_select = shgroup_wire(psl->non_meshes, ts.colorSelect, sh);
		stl->g_data->wire_select_group = shgroup_wire(psl->non_meshes, ts.colorGroupActive, sh);

		/* Transform */
		stl->g_data->wire_transform = shgroup_wire(psl->non_meshes, ts.colorTransform, sh);

		/* Active */
		stl->g_data->wire_active = shgroup_wire(psl->non_meshes, ts.colorActive, sh);
		stl->g_data->wire_active_group = shgroup_wire(psl->non_meshes, ts.colorGroupActive, sh);
	}

	{
		/* Metaballs Handles */
		struct Gwn_Batch *geom;
		geom = DRW_cache_screenspace_circle_get();
		stl->g_data->mball_handle = shgroup_instance_mball_handles(psl->non_meshes, geom);
	}

	{
		/* Lamps */
		/* TODO
		 * for now we create multiple times the same VBO with only lamp center coordinates
		 * but ideally we would only create it once */
		struct Gwn_Batch *geom;

		/* start with buflimit because we don't want stipples */
		geom = DRW_cache_single_line_get();
		stl->g_data->lamp_buflimit = shgroup_distance_lines_instance(psl->non_meshes, geom);

		stl->g_data->lamp_center = shgroup_dynpoints_uniform_color(psl->non_meshes, ts.colorLampNoAlpha, &ts.sizeLampCenter);
		stl->g_data->lamp_center_group = shgroup_dynpoints_uniform_color(psl->non_meshes, ts.colorGroup, &ts.sizeLampCenter);

		geom = DRW_cache_lamp_get();
		stl->g_data->lamp_circle = shgroup_instance_screenspace(psl->non_meshes, geom, &ts.sizeLampCircle);
		geom = DRW_cache_lamp_shadows_get();
		stl->g_data->lamp_circle_shadow = shgroup_instance_screenspace(psl->non_meshes, geom, &ts.sizeLampCircleShadow);

		geom = DRW_cache_lamp_sunrays_get();
		stl->g_data->lamp_sunrays = shgroup_instance_screenspace(psl->non_meshes, geom, &ts.sizeLampCircle);

		stl->g_data->lamp_groundline = shgroup_groundlines_uniform_color(psl->non_meshes, ts.colorLamp);
		stl->g_data->lamp_groundpoint = shgroup_groundpoints_uniform_color(psl->non_meshes, ts.colorLamp);

		geom = DRW_cache_lamp_area_get();
		stl->g_data->lamp_area = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_lamp_hemi_get();
		stl->g_data->lamp_hemi = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_single_line_get();
		stl->g_data->lamp_distance = shgroup_distance_lines_instance(psl->non_meshes, geom);

		geom = DRW_cache_single_line_endpoints_get();
		stl->g_data->lamp_buflimit_points = shgroup_distance_lines_instance(psl->non_meshes, geom);

		geom = DRW_cache_lamp_spot_get();
		stl->g_data->lamp_spot_cone = shgroup_spot_instance(psl->non_meshes, geom);

		geom = DRW_cache_circle_get();
		stl->g_data->lamp_spot_blend = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_lamp_spot_square_get();
		stl->g_data->lamp_spot_pyramid = shgroup_instance(psl->non_meshes, geom);

		geom = DRW_cache_square_get();
		stl->g_data->lamp_spot_blend_rect = shgroup_instance(psl->non_meshes, geom);
	}

	{
		/* -------- STIPPLES ------- */
		/* TODO port to shader stipple */
		struct Gwn_Batch *geom;

		/* Relationship Lines */
		stl->g_data->relationship_lines = shgroup_dynlines_uniform_color(psl->non_meshes, ts.colorWire);
		DRW_shgroup_state_enable(stl->g_data->relationship_lines, DRW_STATE_STIPPLE_3);

		/* Force Field Curve Guide End (here because of stipple) */
		geom = DRW_cache_screenspace_circle_get();
		stl->g_data->field_curve_end = shgroup_instance_screen_aligned(psl->non_meshes, geom);

		/* Force Field Limits */
		geom = DRW_cache_field_tube_limit_get();
		stl->g_data->field_tube_limit = shgroup_instance_scaled(psl->non_meshes, geom);

		geom = DRW_cache_field_cone_limit_get();
		stl->g_data->field_cone_limit = shgroup_instance_scaled(psl->non_meshes, geom);
	}

	{
		/* Object Center pass grouped by State */
		DRWShadingGroup *grp;
		static float outlineWidth, size;

		DRWState state = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND | DRW_STATE_POINT;
		psl->ob_center = DRW_pass_create("Obj Center Pass", state);

		outlineWidth = 1.0f * U.pixelsize;
		size = U.obcenter_dia * U.pixelsize + outlineWidth;

		GPUShader *sh = GPU_shader_get_builtin_shader(GPU_SHADER_3D_POINT_UNIFORM_SIZE_UNIFORM_COLOR_OUTLINE_AA);

		/* Active */
		grp = DRW_shgroup_point_batch_create(sh, psl->ob_center);
		DRW_shgroup_uniform_float(grp, "size", &size, 1);
		DRW_shgroup_uniform_float(grp, "outlineWidth", &outlineWidth, 1);
		DRW_shgroup_uniform_vec4(grp, "color", ts.colorActive, 1);
		DRW_shgroup_uniform_vec4(grp, "outlineColor", ts.colorOutline, 1);
		stl->g_data->center_active = grp;

		/* Select */
		grp = DRW_shgroup_point_batch_create(sh, psl->ob_center);
		DRW_shgroup_uniform_vec4(grp, "color", ts.colorSelect, 1);
		stl->g_data->center_selected = grp;

		/* Deselect */
		grp = DRW_shgroup_point_batch_create(sh, psl->ob_center);
		DRW_shgroup_uniform_vec4(grp, "color", ts.colorDeselect, 1);
		stl->g_data->center_deselected = grp;

		/* Select (library) */
		grp = DRW_shgroup_point_batch_create(sh, psl->ob_center);
		DRW_shgroup_uniform_vec4(grp, "color", ts.colorLibrarySelect, 1);
		stl->g_data->center_selected_lib = grp;

		/* Deselect (library) */
		grp = DRW_shgroup_point_batch_create(sh, psl->ob_center);
		DRW_shgroup_uniform_vec4(grp, "color", ts.colorLibrary, 1);
		stl->g_data->center_deselected_lib = grp;
	}

	{
		/* Particle Pass */
		psl->particle = DRW_pass_create(
		        "Particle Pass",
		        DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS |
		        DRW_STATE_POINT | DRW_STATE_BLEND);
	}

	{
		/* Empty/Background Image Pass */
		psl->reference_image = DRW_pass_create(
		        "Refrence Image Pass",
		        DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS | DRW_STATE_BLEND);
	}
}

static void DRW_shgroup_mball_handles(OBJECT_StorageList *stl, Object *ob, ViewLayer *view_layer)
{
	MetaBall *mb = ob->data;

	float *color;
	DRW_object_wire_theme_get(ob, view_layer, &color);

	for (MetaElem *ml = mb->elems.first; ml != NULL; ml = ml->next) {
		/* draw radius */
		BKE_mball_element_calc_scale_xform(ml->draw_scale_xform, ob->obmat, &ml->x);
		DRW_shgroup_call_dynamic_add(stl->g_data->mball_handle, ml->draw_scale_xform, &ml->rad, color);
	}
}

static void DRW_shgroup_lamp(OBJECT_StorageList *stl, Object *ob, ViewLayer *view_layer)
{
	Lamp *la = ob->data;
	float *color;
	int theme_id = DRW_object_wire_theme_get(ob, view_layer, &color);
	static float zero = 0.0f;

	typedef struct LampEngineData {
		ObjectEngineData engine_data;
		float shape_mat[4][4];
		float spot_blend_mat[4][4];
	} LampEngineData;

	LampEngineData *lamp_engine_data =
	        (LampEngineData *)DRW_object_engine_data_ensure(
	                ob,
	                &draw_engine_object_type,
	                sizeof(LampEngineData),
	                NULL,
	                NULL);

	float (*shapemat)[4] = lamp_engine_data->shape_mat;
	float (*spotblendmat)[4] = lamp_engine_data->spot_blend_mat;

	/* Don't draw the center if it's selected or active */
	if (theme_id == TH_GROUP)
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_center_group, ob->obmat[3]);
	else if (theme_id == TH_LAMP)
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_center, ob->obmat[3]);

	/* First circle */
	DRW_shgroup_call_dynamic_add(stl->g_data->lamp_circle, ob->obmat[3], color);

	/* draw dashed outer circle if shadow is on. remember some lamps can't have certain shadows! */
	if (la->type != LA_HEMI) {
		if ((la->mode & LA_SHAD_RAY) || ((la->mode & LA_SHAD_BUF) && (la->type == LA_SPOT))) {
			DRW_shgroup_call_dynamic_add(stl->g_data->lamp_circle_shadow, ob->obmat[3], color);
		}
	}

	/* Distance */
	if (ELEM(la->type, LA_HEMI, LA_SUN, LA_AREA)) {
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_distance, color, &zero, &la->dist, ob->obmat);
	}

	copy_m4_m4(shapemat, ob->obmat);

	if (la->type == LA_SUN) {
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_sunrays, ob->obmat[3], color);
	}
	else if (la->type == LA_SPOT) {
		float size[3], sizemat[4][4];
		static float one = 1.0f;
		float blend = 1.0f - pow2f(la->spotblend);

		size[0] = size[1] = sinf(la->spotsize * 0.5f) * la->dist;
		size[2] = cosf(la->spotsize * 0.5f) * la->dist;

		size_to_mat4(sizemat, size);
		mul_m4_m4m4(shapemat, ob->obmat, sizemat);

		size[0] = size[1] = blend; size[2] = 1.0f;
		size_to_mat4(sizemat, size);
		translate_m4(sizemat, 0.0f, 0.0f, -1.0f);
		rotate_m4(sizemat, 'X', (float)(M_PI / 2));
		mul_m4_m4m4(spotblendmat, shapemat, sizemat);

		if (la->mode & LA_SQUARE) {
			DRW_shgroup_call_dynamic_add(stl->g_data->lamp_spot_pyramid, color, &one, shapemat);

			/* hide line if it is zero size or overlaps with outer border,
			 * previously it adjusted to always to show it but that seems
			 * confusing because it doesn't show the actual blend size */
			if (blend != 0.0f && blend != 1.0f) {
				DRW_shgroup_call_dynamic_add(stl->g_data->lamp_spot_blend_rect, color, &one, spotblendmat);
			}
		}
		else {
			DRW_shgroup_call_dynamic_add(stl->g_data->lamp_spot_cone, color, shapemat);

			/* hide line if it is zero size or overlaps with outer border,
			 * previously it adjusted to always to show it but that seems
			 * confusing because it doesn't show the actual blend size */
			if (blend != 0.0f && blend != 1.0f) {
				DRW_shgroup_call_dynamic_add(stl->g_data->lamp_spot_blend, color, &one, spotblendmat);
			}
		}

		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_buflimit, color, &la->clipsta, &la->clipend, ob->obmat);
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_buflimit_points, color, &la->clipsta, &la->clipend, ob->obmat);
	}
	else if (la->type == LA_HEMI) {
		static float hemisize = 2.0f;
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_hemi, color, &hemisize, shapemat);
	}
	else if (la->type == LA_AREA) {
		float size[3] = {1.0f, 1.0f, 1.0f}, sizemat[4][4];

		if (la->area_shape == LA_AREA_RECT) {
			size[1] = la->area_sizey / la->area_size;
			size_to_mat4(sizemat, size);
			mul_m4_m4m4(shapemat, shapemat, sizemat);
		}

		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_area, color, &la->area_size, shapemat);
	}

	/* Line and point going to the ground */
	DRW_shgroup_call_dynamic_add(stl->g_data->lamp_groundline, ob->obmat[3]);
	DRW_shgroup_call_dynamic_add(stl->g_data->lamp_groundpoint, ob->obmat[3]);
}

static void DRW_shgroup_camera(OBJECT_StorageList *stl, Object *ob, ViewLayer *view_layer)
{
	const DRWContextState *draw_ctx = DRW_context_state_get();
	View3D *v3d = draw_ctx->v3d;
	Scene *scene = draw_ctx->scene;
	RegionView3D *rv3d = draw_ctx->rv3d;

	Camera *cam = ob->data;
	const bool is_active = (ob == v3d->camera);
	const bool look_through = (is_active && (rv3d->persp == RV3D_CAMOB));
	float *color;
	DRW_object_wire_theme_get(ob, view_layer, &color);

	float vec[4][3], asp[2], shift[2], scale[3], drawsize;

	scale[0] = 1.0f / len_v3(ob->obmat[0]);
	scale[1] = 1.0f / len_v3(ob->obmat[1]);
	scale[2] = 1.0f / len_v3(ob->obmat[2]);

	BKE_camera_view_frame_ex(scene, cam, cam->drawsize, false, scale,
	                         asp, shift, &drawsize, vec);

	/* Frame coords */
	copy_v2_v2(cam->drwcorners[0], vec[0]);
	copy_v2_v2(cam->drwcorners[1], vec[1]);
	copy_v2_v2(cam->drwcorners[2], vec[2]);
	copy_v2_v2(cam->drwcorners[3], vec[3]);

	/* depth */
	cam->drwdepth = vec[0][2];

	/* tria */
	cam->drwtria[0][0] = shift[0] + ((0.7f * drawsize) * scale[0]);
	cam->drwtria[0][1] = shift[1] + ((drawsize * (asp[1] + 0.1f)) * scale[1]);
	cam->drwtria[1][0] = shift[0];
	cam->drwtria[1][1] = shift[1] + ((1.1f * drawsize * (asp[1] + 0.7f)) * scale[1]);

	if (look_through) {
		/* Only draw the frame. */
		DRW_shgroup_call_dynamic_add(
		        stl->g_data->camera_frame, color, cam->drwcorners,
		        &cam->drwdepth, cam->drwtria, ob->obmat);
	}
	else {
		DRW_shgroup_call_dynamic_add(
		        stl->g_data->camera, color, cam->drwcorners,
		        &cam->drwdepth, cam->drwtria, ob->obmat);

		/* Active cam */
		if (is_active) {
			DRW_shgroup_call_dynamic_add(
			        stl->g_data->camera_tria, color,
			        cam->drwcorners, &cam->drwdepth, cam->drwtria, ob->obmat);
		}
	}

	/* draw the rest in normalize object space */
	copy_m4_m4(cam->drwnormalmat, ob->obmat);
	normalize_m4(cam->drwnormalmat);

	if (cam->flag & CAM_SHOWLIMITS) {
		static float col[3] = {0.5f, 0.5f, 0.25f}, col_hi[3] = {1.0f, 1.0f, 0.5f};
		float sizemat[4][4], size[3] = {1.0f, 1.0f, 0.0f};
		float focusdist = BKE_camera_object_dof_distance(ob);

		copy_m4_m4(cam->drwfocusmat, cam->drwnormalmat);
		translate_m4(cam->drwfocusmat, 0.0f, 0.0f, -focusdist);
		size_to_mat4(sizemat, size);
		mul_m4_m4m4(cam->drwfocusmat, cam->drwfocusmat, sizemat);

		DRW_shgroup_call_dynamic_add(
		        stl->g_data->camera_focus, (is_active ? col_hi : col),
		        &cam->drawsize, cam->drwfocusmat);

		DRW_shgroup_call_dynamic_add(
		        stl->g_data->camera_clip, color,
		        &cam->clipsta, &cam->clipend, cam->drwnormalmat);
		DRW_shgroup_call_dynamic_add(
		        stl->g_data->camera_clip_points, (is_active ? col_hi : col),
		        &cam->clipsta, &cam->clipend, cam->drwnormalmat);
	}

	if (cam->flag & CAM_SHOWMIST) {
		World *world = scene->world;

		if (world) {
			static float col[3] = {0.5f, 0.5f, 0.5f}, col_hi[3] = {1.0f, 1.0f, 1.0f};
			world->mistend = world->miststa + world->mistdist;
			DRW_shgroup_call_dynamic_add(
			        stl->g_data->camera_mist, color,
			        &world->miststa, &world->mistend, cam->drwnormalmat);
			DRW_shgroup_call_dynamic_add(
			        stl->g_data->camera_mist_points, (is_active ? col_hi : col),
			        &world->miststa, &world->mistend, cam->drwnormalmat);
		}
	}
}

static void DRW_shgroup_empty(OBJECT_StorageList *stl, OBJECT_PassList *psl, Object *ob, ViewLayer *view_layer)
{
	float *color;
	DRW_object_wire_theme_get(ob, view_layer, &color);

	switch (ob->empty_drawtype) {
		case OB_PLAINAXES:
			DRW_shgroup_call_dynamic_add(stl->g_data->plain_axes, color, &ob->empty_drawsize, ob->obmat);
			break;
		case OB_SINGLE_ARROW:
			DRW_shgroup_call_dynamic_add(stl->g_data->single_arrow, color, &ob->empty_drawsize, ob->obmat);
			DRW_shgroup_call_dynamic_add(stl->g_data->single_arrow_line, color, &ob->empty_drawsize, ob->obmat);
			break;
		case OB_CUBE:
			DRW_shgroup_call_dynamic_add(stl->g_data->cube, color, &ob->empty_drawsize, ob->obmat);
			break;
		case OB_CIRCLE:
			DRW_shgroup_call_dynamic_add(stl->g_data->circle, color, &ob->empty_drawsize, ob->obmat);
			break;
		case OB_EMPTY_SPHERE:
			DRW_shgroup_call_dynamic_add(stl->g_data->sphere, color, &ob->empty_drawsize, ob->obmat);
			break;
		case OB_EMPTY_CONE:
			DRW_shgroup_call_dynamic_add(stl->g_data->cone, color, &ob->empty_drawsize, ob->obmat);
			break;
		case OB_ARROWS:
			DRW_shgroup_call_dynamic_add(stl->g_data->arrows, color, &ob->empty_drawsize, ob->obmat);
			DRW_shgroup_call_dynamic_add(stl->g_data->axis_names, color, &ob->empty_drawsize, ob->obmat);
			break;
		case OB_EMPTY_IMAGE:
			DRW_shgroup_empty_image(stl, psl, ob, color);
			break;
	}
}

static void DRW_shgroup_forcefield(OBJECT_StorageList *stl, Object *ob, ViewLayer *view_layer)
{
	int theme_id = DRW_object_wire_theme_get(ob, view_layer, NULL);
	float *color = DRW_color_background_blend_get(theme_id);
	PartDeflect *pd = ob->pd;
	Curve *cu = (ob->type == OB_CURVE) ? ob->data : NULL;

	/* TODO Move this to depsgraph */
	float tmp[3];
	copy_v3_fl(pd->drawvec1, ob->empty_drawsize);

	switch (pd->forcefield) {
		case PFIELD_WIND:
			pd->drawvec1[2] = pd->f_strength;
			break;
		case PFIELD_VORTEX:
			if (pd->f_strength < 0.0f) {
				pd->drawvec1[1] = -pd->drawvec1[1];
			}
			break;
		case PFIELD_GUIDE:
			if (cu && (cu->flag & CU_PATH) && ob->curve_cache->path && ob->curve_cache->path->data) {
				where_on_path(ob, 0.0f, pd->drawvec1, tmp, NULL, NULL, NULL);
				where_on_path(ob, 1.0f, pd->drawvec2, tmp, NULL, NULL, NULL);
			}
			break;
	}

	if (pd->falloff == PFIELD_FALL_TUBE) {
		pd->drawvec_falloff_max[0] = pd->drawvec_falloff_max[1] = (pd->flag & PFIELD_USEMAXR) ? pd->maxrad : 1.0f;
		pd->drawvec_falloff_max[2] = (pd->flag & PFIELD_USEMAX) ? pd->maxdist : 0.0f;

		pd->drawvec_falloff_min[0] = pd->drawvec_falloff_min[1] = (pd->flag & PFIELD_USEMINR) ? pd->minrad : 1.0f;
		pd->drawvec_falloff_min[2] = (pd->flag & PFIELD_USEMIN) ? pd->mindist : 0.0f;
	}
	else if (pd->falloff == PFIELD_FALL_CONE) {
		float radius, distance;

		radius = DEG2RADF((pd->flag & PFIELD_USEMAXR) ? pd->maxrad : 1.0f);
		distance = (pd->flag & PFIELD_USEMAX) ? pd->maxdist : 0.0f;
		pd->drawvec_falloff_max[0] = pd->drawvec_falloff_max[1] = distance * sinf(radius);
		pd->drawvec_falloff_max[2] = distance * cosf(radius);

		radius = DEG2RADF((pd->flag & PFIELD_USEMINR) ? pd->minrad : 1.0f);
		distance = (pd->flag & PFIELD_USEMIN) ? pd->mindist : 0.0f;

		pd->drawvec_falloff_min[0] = pd->drawvec_falloff_min[1] = distance * sinf(radius);
		pd->drawvec_falloff_min[2] = distance * cosf(radius);
	}
	/* End of things that should go to depthgraph */

	switch (pd->forcefield) {
		case PFIELD_WIND:
			DRW_shgroup_call_dynamic_add(stl->g_data->field_wind, color, &pd->drawvec1, ob->obmat);
			break;
		case PFIELD_FORCE:
			DRW_shgroup_call_dynamic_add(stl->g_data->field_force, color, &pd->drawvec1, ob->obmat);
			break;
		case PFIELD_VORTEX:
			DRW_shgroup_call_dynamic_add(stl->g_data->field_vortex, color, &pd->drawvec1, ob->obmat);
			break;
		case PFIELD_GUIDE:
			if (cu && (cu->flag & CU_PATH) && ob->curve_cache->path && ob->curve_cache->path->data) {
				DRW_shgroup_call_dynamic_add(stl->g_data->field_curve_sta, color, &pd->f_strength, ob->obmat);
				DRW_shgroup_call_dynamic_add(stl->g_data->field_curve_end, color, &pd->f_strength, ob->obmat);
			}
			break;
	}

	if (pd->falloff == PFIELD_FALL_SPHERE) {
		/* as last, guide curve alters it */
		if ((pd->flag & PFIELD_USEMAX) != 0) {
			DRW_shgroup_call_dynamic_add(stl->g_data->field_curve_end, color, &pd->maxdist, ob->obmat);
		}

		if ((pd->flag & PFIELD_USEMIN) != 0) {
			DRW_shgroup_call_dynamic_add(stl->g_data->field_curve_end, color, &pd->mindist, ob->obmat);
		}
	}
	else if (pd->falloff == PFIELD_FALL_TUBE) {
		if (pd->flag & (PFIELD_USEMAX | PFIELD_USEMAXR)) {
			DRW_shgroup_call_dynamic_add(stl->g_data->field_tube_limit, color, &pd->drawvec_falloff_max, ob->obmat);
		}

		if (pd->flag & (PFIELD_USEMIN | PFIELD_USEMINR)) {
			DRW_shgroup_call_dynamic_add(stl->g_data->field_tube_limit, color, &pd->drawvec_falloff_min, ob->obmat);
		}
	}
	else if (pd->falloff == PFIELD_FALL_CONE) {
		if (pd->flag & (PFIELD_USEMAX | PFIELD_USEMAXR)) {
			DRW_shgroup_call_dynamic_add(stl->g_data->field_cone_limit, color, &pd->drawvec_falloff_max, ob->obmat);
		}

		if (pd->flag & (PFIELD_USEMIN | PFIELD_USEMINR)) {
			DRW_shgroup_call_dynamic_add(stl->g_data->field_cone_limit, color, &pd->drawvec_falloff_min, ob->obmat);
		}
	}
}

static void DRW_shgroup_speaker(OBJECT_StorageList *stl, Object *ob, ViewLayer *view_layer)
{
	float *color;
	static float one = 1.0f;
	DRW_object_wire_theme_get(ob, view_layer, &color);

	DRW_shgroup_call_dynamic_add(stl->g_data->speaker, color, &one, ob->obmat);
}

typedef struct OBJECT_LightProbeEngineData {
	ObjectEngineData engine_data;

	float prb_mats[6][4][4];
	float probe_cube_mat[4][4];
	float draw_size;
	float increment_x[3];
	float increment_y[3];
	float increment_z[3];
	float corner[3];
} OBJECT_LightProbeEngineData;

static void DRW_shgroup_lightprobe(OBJECT_StorageList *stl, OBJECT_PassList *psl, Object *ob, ViewLayer *view_layer)
{
	float *color;
	static float one = 1.0f;
	LightProbe *prb = (LightProbe *)ob->data;
	bool do_outlines = ((ob->base_flag & BASE_SELECTED) != 0);
	DRW_object_wire_theme_get(ob, view_layer, &color);

	OBJECT_LightProbeEngineData *prb_data =
	        (OBJECT_LightProbeEngineData *)DRW_object_engine_data_ensure(
	                ob,
	                &draw_engine_object_type,
	                sizeof(OBJECT_LightProbeEngineData),
	                NULL,
	                NULL);

	if ((DRW_state_is_select() || do_outlines) && ((prb->flag & LIGHTPROBE_FLAG_SHOW_DATA) != 0)) {

		if (prb->type == LIGHTPROBE_TYPE_GRID) {
			/* Update transforms */
			float cell_dim[3], half_cell_dim[3];
			cell_dim[0] = 2.0f / (float)(prb->grid_resolution_x);
			cell_dim[1] = 2.0f / (float)(prb->grid_resolution_y);
			cell_dim[2] = 2.0f / (float)(prb->grid_resolution_z);

			mul_v3_v3fl(half_cell_dim, cell_dim, 0.5f);

			/* First cell. */
			copy_v3_fl(prb_data->corner, -1.0f);
			add_v3_v3(prb_data->corner, half_cell_dim);
			mul_m4_v3(ob->obmat, prb_data->corner);

			/* Opposite neighbor cell. */
			copy_v3_fl3(prb_data->increment_x, cell_dim[0], 0.0f, 0.0f);
			add_v3_v3(prb_data->increment_x, half_cell_dim);
			add_v3_fl(prb_data->increment_x, -1.0f);
			mul_m4_v3(ob->obmat, prb_data->increment_x);
			sub_v3_v3(prb_data->increment_x, prb_data->corner);

			copy_v3_fl3(prb_data->increment_y, 0.0f, cell_dim[1], 0.0f);
			add_v3_v3(prb_data->increment_y, half_cell_dim);
			add_v3_fl(prb_data->increment_y, -1.0f);
			mul_m4_v3(ob->obmat, prb_data->increment_y);
			sub_v3_v3(prb_data->increment_y, prb_data->corner);

			copy_v3_fl3(prb_data->increment_z, 0.0f, 0.0f, cell_dim[2]);
			add_v3_v3(prb_data->increment_z, half_cell_dim);
			add_v3_fl(prb_data->increment_z, -1.0f);
			mul_m4_v3(ob->obmat, prb_data->increment_z);
			sub_v3_v3(prb_data->increment_z, prb_data->corner);

			DRWShadingGroup *grp = DRW_shgroup_instance_create(e_data.lightprobe_grid_sh, psl->lightprobes,
			                                                   DRW_cache_sphere_get(), NULL);
			DRW_shgroup_set_instance_count(grp, prb->grid_resolution_x * prb->grid_resolution_y * prb->grid_resolution_z);
			DRW_shgroup_uniform_vec4(grp, "color", color, 1);
			DRW_shgroup_uniform_vec3(grp, "corner", prb_data->corner, 1);
			DRW_shgroup_uniform_vec3(grp, "increment_x", prb_data->increment_x, 1);
			DRW_shgroup_uniform_vec3(grp, "increment_y", prb_data->increment_y, 1);
			DRW_shgroup_uniform_vec3(grp, "increment_z", prb_data->increment_z, 1);
			DRW_shgroup_uniform_ivec3(grp, "grid_resolution", &prb->grid_resolution_x, 1);
			DRW_shgroup_uniform_float(grp, "sphere_size", &prb->data_draw_size, 1);
		}
		else if (prb->type == LIGHTPROBE_TYPE_CUBE) {
			prb_data->draw_size = prb->data_draw_size * 0.1f;
			unit_m4(prb_data->probe_cube_mat);
			copy_v3_v3(prb_data->probe_cube_mat[3], ob->obmat[3]);
			DRW_shgroup_call_dynamic_add(stl->g_data->lightprobes_cube, color, &prb_data->draw_size, prb_data->probe_cube_mat);
		}
		else {
			prb_data->draw_size = 1.0f;
			DRW_shgroup_call_dynamic_add(stl->g_data->lightprobes_planar, color, &prb_data->draw_size, ob->obmat);
		}
	}

	switch (prb->type) {
		case LIGHTPROBE_TYPE_PLANAR:
			DRW_shgroup_call_dynamic_add(stl->g_data->probe_planar, ob->obmat[3], color);
			break;
		case LIGHTPROBE_TYPE_GRID:
			DRW_shgroup_call_dynamic_add(stl->g_data->probe_grid, ob->obmat[3], color);
			break;
		case LIGHTPROBE_TYPE_CUBE:
		default:
			DRW_shgroup_call_dynamic_add(stl->g_data->probe_cube, ob->obmat[3], color);
			break;
	}



	if (prb->type == LIGHTPROBE_TYPE_PLANAR) {
		float (*mat)[4];
		mat = (float (*)[4])(prb_data->prb_mats[0]);
		copy_m4_m4(mat, ob->obmat);
		normalize_m4(mat);

		DRW_shgroup_call_dynamic_add(stl->g_data->single_arrow, color, &ob->empty_drawsize, mat);
		DRW_shgroup_call_dynamic_add(stl->g_data->single_arrow_line, color, &ob->empty_drawsize, mat);

		mat = (float (*)[4])(prb_data->prb_mats[1]);
		copy_m4_m4(mat, ob->obmat);
		zero_v3(mat[2]);

		DRW_shgroup_call_dynamic_add(stl->g_data->cube, color, &one, mat);
	}

	if ((prb->flag & LIGHTPROBE_FLAG_SHOW_INFLUENCE) != 0) {

		prb->distfalloff = (1.0f - prb->falloff) * prb->distinf;
		prb->distgridinf = prb->distinf;

		if (prb->type == LIGHTPROBE_TYPE_GRID) {
			prb->distfalloff += 1.0f;
			prb->distgridinf += 1.0f;
		}

		if (prb->type == LIGHTPROBE_TYPE_GRID ||
		    prb->attenuation_type == LIGHTPROBE_SHAPE_BOX)
		{
			DRW_shgroup_call_dynamic_add(stl->g_data->cube, color, &prb->distgridinf, ob->obmat);
			DRW_shgroup_call_dynamic_add(stl->g_data->cube, color, &prb->distfalloff, ob->obmat);
		}
		else if (prb->type == LIGHTPROBE_TYPE_PLANAR) {
			float (*rangemat)[4];
			rangemat = (float (*)[4])(prb_data->prb_mats[2]);
			copy_m4_m4(rangemat, ob->obmat);
			normalize_v3(rangemat[2]);
			mul_v3_fl(rangemat[2], prb->distinf);

			DRW_shgroup_call_dynamic_add(stl->g_data->cube, color, &one, rangemat);

			rangemat = (float (*)[4])(prb_data->prb_mats[3]);
			copy_m4_m4(rangemat, ob->obmat);
			normalize_v3(rangemat[2]);
			mul_v3_fl(rangemat[2], prb->distfalloff);

			DRW_shgroup_call_dynamic_add(stl->g_data->cube, color, &one, rangemat);
		}
		else {
			DRW_shgroup_call_dynamic_add(stl->g_data->sphere, color, &prb->distgridinf, ob->obmat);
			DRW_shgroup_call_dynamic_add(stl->g_data->sphere, color, &prb->distfalloff, ob->obmat);
		}
	}

	if ((prb->flag & LIGHTPROBE_FLAG_SHOW_PARALLAX) != 0) {
		if (prb->type != LIGHTPROBE_TYPE_PLANAR) {
			float (*obmat)[4], *dist;

			if ((prb->flag & LIGHTPROBE_FLAG_CUSTOM_PARALLAX) != 0) {
				dist = &prb->distpar;
				/* TODO object parallax */
				obmat = ob->obmat;
			}
			else {
				dist = &prb->distinf;
				obmat = ob->obmat;
			}

			if (prb->parallax_type == LIGHTPROBE_SHAPE_BOX) {
				DRW_shgroup_call_dynamic_add(stl->g_data->cube, color, dist, obmat);
			}
			else {
				DRW_shgroup_call_dynamic_add(stl->g_data->sphere, color, dist, obmat);
			}
		}
	}

	if ((prb->flag & LIGHTPROBE_FLAG_SHOW_CLIP_DIST) != 0) {
		if (prb->type != LIGHTPROBE_TYPE_PLANAR) {
			static const float cubefacemat[6][4][4] = {
				{{0.0, 0.0, -1.0, 0.0}, {0.0, -1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
				{{0.0, 0.0, 1.0, 0.0}, {0.0, -1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
				{{1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, -1.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
				{{1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, -1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
				{{1.0, 0.0, 0.0, 0.0}, {0.0, -1.0, 0.0, 0.0}, {0.0, 0.0, -1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
				{{-1.0, 0.0, 0.0, 0.0}, {0.0, -1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}},
			};

			for (int i = 0; i < 6; ++i) {
				float (*clipmat)[4];
				clipmat = (float (*)[4])(prb_data->prb_mats[i]);

				normalize_m4_m4(clipmat, ob->obmat);
				mul_m4_m4m4(clipmat, clipmat, cubefacemat[i]);

				DRW_shgroup_call_dynamic_add(stl->g_data->lamp_buflimit, color, &prb->clipsta, &prb->clipend, clipmat);
				DRW_shgroup_call_dynamic_add(stl->g_data->lamp_buflimit_points, color, &prb->clipsta, &prb->clipend, clipmat);
			}
		}
	}

	/* Line and point going to the ground */
	if (prb->type == LIGHTPROBE_TYPE_CUBE) {
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_groundline, ob->obmat[3]);
		DRW_shgroup_call_dynamic_add(stl->g_data->lamp_groundpoint, ob->obmat[3]);
	}
}

static void DRW_shgroup_relationship_lines(OBJECT_StorageList *stl, Object *ob)
{
	if (ob->parent && DRW_check_object_visible_within_active_context(ob->parent)) {
		DRW_shgroup_call_dynamic_add(stl->g_data->relationship_lines, ob->obmat[3]);
		DRW_shgroup_call_dynamic_add(stl->g_data->relationship_lines, ob->parent->obmat[3]);
	}
}

static void DRW_shgroup_object_center(OBJECT_StorageList *stl, Object *ob, ViewLayer *view_layer, View3D *v3d)
{
	const bool is_library = ob->id.us > 1 || ID_IS_LINKED(ob);
	DRWShadingGroup *shgroup;

	if (ob == OBACT(view_layer)) {
		shgroup = stl->g_data->center_active;
	}
	else if (ob->base_flag & BASE_SELECTED) {
		if (is_library) {
			shgroup = stl->g_data->center_selected_lib;
		}
		else {
			shgroup = stl->g_data->center_selected;
		}
	}
	else if (v3d->flag & V3D_DRAW_CENTERS) {
		if (is_library) {
			shgroup = stl->g_data->center_deselected_lib;
		}
		else {
			shgroup = stl->g_data->center_deselected;
		}
	}
	else {
		return;
	}

	DRW_shgroup_call_dynamic_add(shgroup, ob->obmat[3]);
}

static void OBJECT_cache_populate_particles(Object *ob,
                                            OBJECT_PassList *psl)
{
	for (ParticleSystem *psys = ob->particlesystem.first; psys; psys = psys->next) {
		if (psys_check_enabled(ob, psys, false)) {
			ParticleSettings *part = psys->part;
			int draw_as = (part->draw_as == PART_DRAW_REND) ? part->ren_as : part->draw_as;

			if (draw_as == PART_DRAW_PATH && !psys->pathcache && !psys->childcache) {
				draw_as = PART_DRAW_DOT;
			}

			static float mat[4][4];
			unit_m4(mat);

			if (draw_as != PART_DRAW_PATH) {
				struct Gwn_Batch *geom = DRW_cache_particles_get_dots(psys);
				DRWShadingGroup *shgrp = NULL;
				static int screen_space[2] = {0, 1};
				static float def_prim_col[3] = {0.5f, 0.5f, 0.5f};
				static float def_sec_col[3] = {1.0f, 1.0f, 1.0f};

				Material *ma = give_current_material(ob, part->omat);

				switch (draw_as) {
					case PART_DRAW_DOT:
						shgrp = DRW_shgroup_create(e_data.part_dot_sh, psl->particle);
						DRW_shgroup_uniform_vec3(shgrp, "color", ma ? &ma->r : def_prim_col, 1);
						DRW_shgroup_uniform_vec3(shgrp, "outlineColor", ma ? &ma->specr : def_sec_col, 1);
						DRW_shgroup_uniform_float(shgrp, "pixel_size", DRW_viewport_pixelsize_get(), 1);
						DRW_shgroup_uniform_float(shgrp, "size", &part->draw_size, 1);
						DRW_shgroup_uniform_texture(shgrp, "ramp", globals_ramp);
						DRW_shgroup_call_add(shgrp, geom, mat);
						break;
					case PART_DRAW_CROSS:
						shgrp = DRW_shgroup_instance_create(
						        e_data.part_prim_sh, psl->particle, DRW_cache_particles_get_prim(PART_DRAW_CROSS), NULL);
						DRW_shgroup_uniform_texture(shgrp, "ramp", globals_ramp);
						DRW_shgroup_uniform_vec3(shgrp, "color", ma ? &ma->r : def_prim_col, 1);
						DRW_shgroup_uniform_int(shgrp, "screen_space", &screen_space[0], 1);
						break;
					case PART_DRAW_CIRC:
						shgrp = DRW_shgroup_instance_create(
						        e_data.part_prim_sh, psl->particle, DRW_cache_particles_get_prim(PART_DRAW_CIRC), NULL);
						DRW_shgroup_uniform_texture(shgrp, "ramp", globals_ramp);
						DRW_shgroup_uniform_vec3(shgrp, "color", ma ? &ma->r : def_prim_col, 1);
						DRW_shgroup_uniform_int(shgrp, "screen_space", &screen_space[1], 1);
						break;
					case PART_DRAW_AXIS:
						shgrp = DRW_shgroup_instance_create(
						        e_data.part_axis_sh, psl->particle, DRW_cache_particles_get_prim(PART_DRAW_AXIS), NULL);
						DRW_shgroup_uniform_int(shgrp, "screen_space", &screen_space[0], 1);
						break;
					default:
						break;
				}

				if (shgrp) {
					if (draw_as != PART_DRAW_DOT) {
						DRW_shgroup_uniform_float(shgrp, "draw_size", &part->draw_size, 1);
						DRW_shgroup_instance_batch(shgrp, geom);
					}
				}
			}
		}
	}
}

static void OBJECT_cache_populate(void *vedata, Object *ob)
{
	OBJECT_PassList *psl = ((OBJECT_Data *)vedata)->psl;
	OBJECT_StorageList *stl = ((OBJECT_Data *)vedata)->stl;
	const DRWContextState *draw_ctx = DRW_context_state_get();
	ViewLayer *view_layer = draw_ctx->view_layer;
	View3D *v3d = draw_ctx->v3d;
	int theme_id = TH_UNDEFINED;

	/* Handle particles first in case the emitter itself shouldn't be rendered. */
	if (ob->type == OB_MESH) {
		OBJECT_cache_populate_particles(ob, psl);
	}

	if (DRW_check_object_visible_within_active_context(ob) == false) {
		return;
	}

	//CollectionEngineSettings *ces_mode_ob = BKE_layer_collection_engine_evaluated_get(ob, COLLECTION_MODE_OBJECT, "");

	//bool do_wire = BKE_collection_engine_property_value_get_bool(ces_mode_ob, "show_wire");
	bool do_outlines = ((ob->base_flag & BASE_SELECTED) != 0);

	if (do_outlines) {
		if ((ob != draw_ctx->object_edit) && !((ob == draw_ctx->obact) && (draw_ctx->object_mode & OB_MODE_ALL_PAINT))) {
			struct Gwn_Batch *geom = DRW_cache_object_surface_get(ob);
			if (geom) {
				theme_id = DRW_object_wire_theme_get(ob, view_layer, NULL);
				DRWShadingGroup *shgroup = shgroup_theme_id_to_outline_or(stl, theme_id, NULL);
				if (shgroup != NULL) {
					DRW_shgroup_call_add(shgroup, geom, ob->obmat);
				}
			}
		}
	}

	switch (ob->type) {
		case OB_MESH:
		{
			Mesh *me = ob->data;
			if (me->totpoly == 0) {
				if (ob != draw_ctx->object_edit) {
					struct Gwn_Batch *geom = DRW_cache_mesh_edges_get(ob);
					if (geom) {
						if (theme_id == TH_UNDEFINED) {
							theme_id = DRW_object_wire_theme_get(ob, view_layer, NULL);
						}

						DRWShadingGroup *shgroup = shgroup_theme_id_to_wire_or(stl, theme_id, stl->g_data->wire);
						DRW_shgroup_call_add(shgroup, geom, ob->obmat);
					}
				}
			}
			break;
		}
		case OB_SURF:
			break;
		case OB_LATTICE:
		{
			if (ob != draw_ctx->object_edit) {
				struct Gwn_Batch *geom = DRW_cache_lattice_wire_get(ob, false);
				if (theme_id == TH_UNDEFINED) {
					theme_id = DRW_object_wire_theme_get(ob, view_layer, NULL);
				}

				DRWShadingGroup *shgroup = shgroup_theme_id_to_wire_or(stl, theme_id, stl->g_data->wire);
				DRW_shgroup_call_add(shgroup, geom, ob->obmat);
			}
			break;
		}

		case OB_CURVE:
		{
			if (ob != draw_ctx->object_edit) {
				struct Gwn_Batch *geom = DRW_cache_curve_edge_wire_get(ob);
				if (theme_id == TH_UNDEFINED) {
					theme_id = DRW_object_wire_theme_get(ob, view_layer, NULL);
				}
				DRWShadingGroup *shgroup = shgroup_theme_id_to_wire_or(stl, theme_id, stl->g_data->wire);
				DRW_shgroup_call_add(shgroup, geom, ob->obmat);
			}
			break;
		}
		case OB_MBALL:
		{
			if (ob != draw_ctx->object_edit) {
				DRW_shgroup_mball_handles(stl, ob, view_layer);
			}
			break;
		}
		case OB_LAMP:
			DRW_shgroup_lamp(stl, ob, view_layer);
			break;
		case OB_CAMERA:
			DRW_shgroup_camera(stl, ob, view_layer);
			break;
		case OB_EMPTY:
			DRW_shgroup_empty(stl, psl, ob, view_layer);
			break;
		case OB_SPEAKER:
			DRW_shgroup_speaker(stl, ob, view_layer);
			break;
		case OB_LIGHTPROBE:
			DRW_shgroup_lightprobe(stl, psl, ob, view_layer);
			break;
		case OB_ARMATURE:
		{
			bArmature *arm = ob->data;
			if (arm->edbo == NULL) {
				if (DRW_state_is_select() || !DRW_pose_mode_armature(ob, draw_ctx->obact)) {
					DRW_shgroup_armature_object(
					        ob, view_layer, psl->bone_solid, psl->bone_wire, psl->bone_envelope,
					        stl->g_data->relationship_lines);
				}
			}
			break;
		}
		default:
			break;
	}

	if (ob->pd && ob->pd->forcefield) {
		DRW_shgroup_forcefield(stl, ob, view_layer);
	}

	/* don't show object extras in set's */
	if ((ob->base_flag & (BASE_FROM_SET | BASE_FROMDUPLI)) == 0) {

		DRW_shgroup_object_center(stl, ob, view_layer, v3d);

		DRW_shgroup_relationship_lines(stl, ob);

		if ((ob->dtx & OB_DRAWNAME) && DRW_state_show_text()) {
			struct DRWTextStore *dt = DRW_text_cache_ensure();
			if (theme_id == TH_UNDEFINED) {
				theme_id = DRW_object_wire_theme_get(ob, view_layer, NULL);
			}

			unsigned char color[4];
			UI_GetThemeColor4ubv(theme_id, color);

			DRW_text_cache_add(
			        dt, ob->obmat[3],
			        ob->id.name + 2, strlen(ob->id.name + 2),
			        10, DRW_TEXT_CACHE_GLOBALSPACE | DRW_TEXT_CACHE_STRING_PTR, color);
		}
	}
}

static void OBJECT_draw_scene(void *vedata)
{

	OBJECT_PassList *psl = ((OBJECT_Data *)vedata)->psl;
	OBJECT_StorageList *stl = ((OBJECT_Data *)vedata)->stl;
	OBJECT_FramebufferList *fbl = ((OBJECT_Data *)vedata)->fbl;
	DefaultFramebufferList *dfbl = DRW_viewport_framebuffer_list_get();
	DefaultTextureList *dtxl = DRW_viewport_texture_list_get();

	float clearcol[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	if (DRW_state_is_fbo()) {
		DRW_stats_group_start("Outlines");
		/* attach temp textures */
		DRW_framebuffer_texture_attach(fbl->outlines, e_data.outlines_depth_tx, 0, 0);
		DRW_framebuffer_texture_attach(fbl->outlines, e_data.outlines_color_tx, 0, 0);
		DRW_framebuffer_texture_attach(fbl->blur, e_data.outlines_blur_tx, 0, 0);
		
		/* Render filled polygon on a separate framebuffer */
		DRW_framebuffer_bind(fbl->outlines);
		DRW_framebuffer_clear(true, true, false, clearcol, 1.0f);
		DRW_draw_pass(psl->outlines);
		DRW_draw_pass(psl->lightprobes);

		/* detach textures */
		DRW_framebuffer_texture_detach(e_data.outlines_depth_tx);

		/* Search outline pixels */
		DRW_framebuffer_bind(fbl->blur);
		DRW_draw_pass(psl->outlines_search);

		/* Expand outline to form a 3px wide line */
		DRW_framebuffer_bind(fbl->outlines);
		DRW_draw_pass(psl->outlines_expand);

		/* Bleed color so the AA can do it's stuff */
		DRW_framebuffer_bind(fbl->blur);
		DRW_draw_pass(psl->outlines_bleed);

		/* detach temp textures */
		DRW_framebuffer_texture_detach(e_data.outlines_color_tx);
		DRW_framebuffer_texture_detach(e_data.outlines_blur_tx);

		/* restore main framebuffer */
		DRW_framebuffer_bind(dfbl->default_fb);
		DRW_stats_group_end();
	}
	else if (DRW_state_is_select()) {
		/* Render probes spheres/planes so we can select them. */
		DRW_draw_pass(psl->lightprobes);
	}

	MULTISAMPLE_SYNC_ENABLE(dfbl)

	/* This needs to be drawn after the oultine */
//	DRW_draw_pass(psl->bone_envelope);  /* Never drawn in Object mode currently. */
	DRW_draw_pass(psl->bone_wire);
	DRW_draw_pass(psl->bone_solid);
	DRW_draw_pass(psl->non_meshes);
	DRW_draw_pass(psl->particle);
	DRW_draw_pass(psl->reference_image);

	MULTISAMPLE_SYNC_DISABLE(dfbl)

	DRW_draw_pass(psl->ob_center);

	if (DRW_state_is_fbo()) {
		if (e_data.draw_grid) {
			DRW_framebuffer_texture_detach(dtxl->depth);
			DRW_draw_pass(psl->grid);
			DRW_framebuffer_texture_attach(dfbl->default_fb, dtxl->depth, 0, 0);
		}

		/* Combine with scene buffer last */
		DRW_draw_pass(psl->outlines_resolve);
	}

	/* This has to be freed only after drawing empties! */
	if (stl->g_data->image_plane_map) {
		BLI_ghash_free(stl->g_data->image_plane_map, NULL, MEM_freeN);
	}
}

void OBJECT_collection_settings_create(IDProperty *props)
{
	BLI_assert(props &&
	           props->type == IDP_GROUP &&
	           props->subtype == IDP_GROUP_SUB_MODE_OBJECT);
	BKE_collection_engine_property_add_int(props, "show_wire", false);
	BKE_collection_engine_property_add_int(props, "show_backface_culling", false);
}

static const DrawEngineDataSize OBJECT_data_size = DRW_VIEWPORT_DATA_SIZE(OBJECT_Data);

DrawEngineType draw_engine_object_type = {
	NULL, NULL,
	N_("ObjectMode"),
	&OBJECT_data_size,
	&OBJECT_engine_init,
	&OBJECT_engine_free,
	&OBJECT_cache_init,
	&OBJECT_cache_populate,
	NULL,
	NULL,
	&OBJECT_draw_scene,
	NULL,
	NULL,
};
