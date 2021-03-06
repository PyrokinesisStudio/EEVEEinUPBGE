# ***** BEGIN GPL LICENSE BLOCK *****
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public LicenseS
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# Contributor(s): Jacques Beaurain.
#
# ***** END GPL LICENSE BLOCK *****

set(INC

)

set(INC_SYS

)

# blender and player
set(SRC

)

# Order matches "UI_icons.h", final name will be formatted: "icons{size}_{name}.dat"
set(ICON_NAMES
	question
	error
	cancel
	tria_right
	tria_down
	tria_left
	tria_up
	arrow_leftright
	plus
	disclosure_tri_down
	disclosure_tri_right
	radiobut_off
	radiobut_on
	menu_panel
	blender
	grip
	dot
	collapsemenu
	x
	go_left
	plug
	ui
	node
	node_sel
	fullscreen
	splitscreen
	rightarrow_thin
	bordermove
	viewzoom
	zoomin
	zoomout
	panel_close
	copy_id
	eyedropper
	link_area
	auto
	checkbox_dehlt
	checkbox_hlt
	unlocked
	locked
	unpinned
	pinned
	screen_back
	rightarrow
	downarrow_hlt
	dotsup
	dotsdown
	link
	inlink
	plugin
	help
	ghost_enabled
	color
	linked
	unlinked
	hand
	zoom_all
	zoom_selected
	zoom_previous
	zoom_in
	zoom_out
	render_region
	border_rect
	border_lasso
	freeze
	stylus_pressure
	ghost_disabled
	new
	file_tick
	quit
	url
	recover_last
	fullscreen_enter
	fullscreen_exit
	lamp
	material
	texture
	anim
	world
	scene
	edit
	game
	radio
	script
	particles
	physics
	speaker
	texture_shaded
	view3d
	ipo
	oops
	buts
	filesel
	image_col
	info
	sequence
	text
	imasel
	sound
	action
	nla
	scriptwin
	time
	nodetree
	logic
	console
	preferences
	clip
	asset_manager
	object_datamode
	editmode_hlt
	facesel_hlt
	vpaint_hlt
	tpaint_hlt
	wpaint_hlt
	sculptmode_hlt
	pose_hlt
	particlemode
	lightpaint
	greasepencil_stroke_paint
	scene_data
	renderlayers
	world_data
	object_data
	mesh_data
	curve_data
	meta_data
	lattice_data
	lamp_data
	material_data
	texture_data
	anim_data
	camera_data
	particle_data
	library_data_direct
	group
	armature_data
	pose_data
	bone_data
	constraint
	shapekey_data
	constraint_bone
	camera_stereo
	package
	uglypackage
	brush_data
	image_data
	file
	fcurve
	font_data
	render_result
	surface_data
	empty_data
	settings
	render_animation
	render_still
	library_data_broken
	boids
	strands
	library_data_indirect
	greasepencil
	line_data
	library_data_override
	group_bone
	group_vertex
	group_vcol
	group_uvs
	rna
	rna_add
	outliner_ob_empty
	outliner_ob_mesh
	outliner_ob_curve
	outliner_ob_lattice
	outliner_ob_meta
	outliner_ob_lamp
	outliner_ob_camera
	outliner_ob_armature
	outliner_ob_font
	outliner_ob_surface
	outliner_ob_speaker
	outliner_ob_force_field
	outliner_ob_group_instance
	outliner_ob_greasepencil
	restrict_color_off
	restrict_color_on
	restrict_view_off
	restrict_view_on
	restrict_select_off
	restrict_select_on
	restrict_render_off
	restrict_render_on
	outliner_data_empty
	outliner_data_mesh
	outliner_data_curve
	outliner_data_lattice
	outliner_data_meta
	outliner_data_lamp
	outliner_data_camera
	outliner_data_armature
	outliner_data_font
	outliner_data_surface
	outliner_data_speaker
	outliner_data_pose
	outliner_data_greasepencil
	mesh_plane
	mesh_cube
	mesh_circle
	mesh_uvsphere
	mesh_icosphere
	mesh_grid
	mesh_monkey
	mesh_cylinder
	mesh_torus
	mesh_cone
	mesh_capsule
	lamp_point
	lamp_sun
	lamp_spot
	lamp_hemi
	lamp_area
	meta_empty
	meta_plane
	meta_cube
	meta_ball
	meta_ellipsoid
	meta_capsule
	surface_ncurve
	surface_ncircle
	surface_nsurface
	surface_ncylinder
	surface_nsphere
	surface_ntorus
	curve_bezcurve
	curve_bezcircle
	curve_ncurve
	curve_ncircle
	curve_path
	color_red
	color_green
	color_blue
	tria_right_bar
	tria_down_bar
	tria_left_bar
	tria_up_bar
	force_force
	force_wind
	force_vortex
	force_magnetic
	force_harmonic
	force_charge
	force_lennardjones
	force_texture
	force_curve
	force_boid
	force_turbulence
	force_drag
	force_smokeflow
	node_insert_on
	node_insert_off
	modifier
	mod_wave
	mod_build
	mod_decim
	mod_mirror
	mod_soft
	mod_subsurf
	hook
	mod_physics
	mod_particles
	mod_boolean
	mod_edgesplit
	mod_array
	mod_uvproject
	mod_displace
	mod_curve
	mod_lattice
	constraint_data
	mod_armature
	mod_shrinkwrap
	mod_cast
	mod_meshdeform
	mod_bevel
	mod_smooth
	mod_simpledeform
	mod_mask
	mod_cloth
	mod_explode
	mod_fluidsim
	mod_multires
	mod_smoke
	mod_solidify
	mod_screw
	mod_vertex_weight
	mod_dynamicpaint
	mod_remesh
	mod_ocean
	mod_warp
	mod_skin
	mod_triangulate
	mod_wireframe
	mod_data_transfer
	mod_normaledit
	rec
	play
	ff
	rew
	pause
	prev_keyframe
	next_keyframe
	play_audio
	play_reverse
	preview_range
	action_tweak
	pmarker_act
	pmarker_sel
	pmarker
	marker_hlt
	marker
	space2
	space3
	keyingset
	key_dehlt
	key_hlt
	mute_ipo_off
	mute_ipo_on
	visible_ipo_off
	visible_ipo_on
	driver
	solo_off
	solo_on
	frame_prev
	frame_next
	nla_pushdown
	ipo_constant
	ipo_linear
	ipo_bezier
	ipo_sine
	ipo_quad
	ipo_cubic
	ipo_quart
	ipo_quint
	ipo_expo
	ipo_circ
	ipo_bounce
	ipo_elastic
	ipo_back
	ipo_ease_in
	ipo_ease_out
	ipo_ease_in_out
	normalize_fcurves
	vertexsel
	edgesel
	facesel
	loopsel
	rotate
	cursor
	rotatecollection
	rotatecenter
	rotactive
	align
	smoothcurve
	spherecurve
	rootcurve
	sharpcurve
	lincurve
	nocurve
	rndcurve
	prop_off
	prop_on
	prop_con
	particle_point
	particle_tip
	particle_path
	man_trans
	man_rot
	man_scale
	manipul
	snap_off
	snap_on
	snap_normal
	snap_grid
	snap_vertex
	snap_edge
	snap_face
	snap_volume
	snap_increment
	sticky_uvs_loc
	sticky_uvs_disable
	sticky_uvs_vert
	clipuv_dehlt
	clipuv_hlt
	snap_peel_object
	grid
	object_origin
	pastedown
	copydown
	pasteflipup
	pasteflipdown
	snap_surface
	automerge_on
	automerge_off
	retopo
	uv_vertexsel
	uv_edgesel
	uv_facesel
	uv_islandsel
	uv_sync_select
	bbox
	wire
	solid
	smooth
	potato
	ortho
	lockview_off
	lockview_on
	axis_side
	axis_front
	axis_top
	ndof_dom
	ndof_turn
	ndof_fly
	ndof_trans
	layer_used
	layer_active
	sortalpha
	sortbyext
	sorttime
	sortsize
	longdisplay
	shortdisplay
	ghost
	imgdisplay
	save_as
	save_copy
	bookmarks
	fontpreview
	filter
	newfolder
	open_recent
	file_parent
	file_refresh
	file_folder
	file_blend
	file_image
	file_movie
	file_script
	file_sound
	file_font
	file_text
	recover_auto
	save_prefs
	link_blend
	append_blend
	import
	export
	external_data
	load_factory
	loop_back
	loop_forwards
	back
	forward
	file_hidden
	file_backup
	disk_drive
	matplane
	matsphere
	matcube
	monkey
	hair
	aliased
	antialiased
	mat_sphere_sky
	wordwrap_off
	wordwrap_on
	syntax_off
	syntax_on
	linenumbers_off
	linenumbers_on
	scriptplugins
	seq_sequencer
	seq_preview
	seq_luma_waveform
	seq_chroma_scope
	seq_histogram
	seq_splitview
	image_rgb
	image_rgb_alpha
	image_alpha
	image_zdepth
	imagefile
)

data_to_c_simple(../../../../release/datafiles/bfont.pfb SRC)
data_to_c_simple(../../../../release/datafiles/bfont.ttf SRC)
data_to_c_simple(../../../../release/datafiles/bmonofont.ttf SRC)

if(WITH_BLENDER)
	# blender only (not player)

	if(NOT WITH_HEADLESS)
		# blender UI only

		# blends
		data_to_c_simple(../../../../release/datafiles/preview.blend SRC)
		data_to_c_simple(../../../../release/datafiles/preview_cycles.blend SRC)

		# images
		data_to_c_simple(../../../../release/datafiles/splash.png SRC)
		data_to_c_simple(../../../../release/datafiles/splash_2x.png SRC)
		# XXX These are handy, but give nasty "false changes" in svn :/
		#svg_to_png(../../../../release/datafiles/blender_icons.svg
		           #../../../../release/datafiles/blender_icons16.png
		           #90 SRC)

		data_to_c_simple_icons(../../../../release/datafiles/blender_icons16 "icon16_" "${ICON_NAMES}" SRC)
		#data_to_c_simple(../../../../release/datafiles/blender_icons16.png SRC)
		#svg_to_png(../../../../release/datafiles/blender_icons.svg
		           #../../../../release/datafiles/blender_icons32.png
		           #180 SRC)
		data_to_c_simple_icons(../../../../release/datafiles/blender_icons32 "icon32_" "${ICON_NAMES}" SRC)
		#data_to_c_simple(../../../../release/datafiles/blender_icons32.png SRC)
		#svg_to_png(../../../../release/datafiles/prvicons.svg
		           #../../../../release/datafiles/prvicons.png
		           #90 SRC)
		data_to_c_simple(../../../../release/datafiles/prvicons.png SRC)

		# brushes
		data_to_c_simple(../../../../release/datafiles/brushicons/add.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/blob.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/blur.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/clay.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/claystrips.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/clone.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/crease.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/darken.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/draw.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/fill.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/flatten.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/grab.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/inflate.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/layer.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/lighten.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/mask.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/mix.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/multiply.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/nudge.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/pinch.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/scrape.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/smear.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/smooth.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/snake_hook.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/soften.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/subtract.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/texdraw.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/texfill.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/texmask.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/thumb.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/twist.png SRC)
		data_to_c_simple(../../../../release/datafiles/brushicons/vertexdraw.png SRC)

		# matcap
		data_to_c_simple(../../../../release/datafiles/matcaps/mc01.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc02.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc03.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc04.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc05.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc06.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc07.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc08.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc09.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc10.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc11.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc12.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc13.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc14.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc15.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc16.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc17.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc18.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc19.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc20.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc21.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc22.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc23.jpg SRC)
		data_to_c_simple(../../../../release/datafiles/matcaps/mc24.jpg SRC)

	endif()

	data_to_c_simple(../../../../release/datafiles/startup.blend SRC)
endif()

unset(ICON_NAMES)

blender_add_lib(bf_editor_datafiles "${SRC}" "${INC}" "${INC_SYS}")
