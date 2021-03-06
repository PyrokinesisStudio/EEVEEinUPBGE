# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# <pep8 compliant>
import bpy
from bpy.types import Panel


class CollectionButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "collection"


def get_collection_from_context(context):
    active_object = context.active_object

    if active_object and active_object.dupli_group and context.space_data.collection_context == 'GROUP':
        group = active_object.dupli_group
        return group.view_layer.collections.active
    else:
        return context.layer_collection


class COLLECTION_PT_context_collection(CollectionButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}

    def draw(self, context):
        layout = self.layout
        space = context.space_data
        active_object = context.active_object

        if active_object and active_object.dupli_group:
            split = layout.split(percentage=0.2)
            split.row().prop(space, "collection_context", expand=True)
            layout = split

        collection = get_collection_from_context(context)
        name = collection.name
        if name == 'Master Collection':
            layout.label(text=name, icon='COLLAPSEMENU')
        else:
            layout.prop(collection, "name", text="", icon='COLLAPSEMENU')


class COLLECTION_PT_clay_settings(CollectionButtonsPanel, Panel):
    bl_label = "Render Settings"
    COMPAT_ENGINES = {'BLENDER_CLAY'}

    @classmethod
    def poll(cls, context):
        return context.view_render.engine in cls.COMPAT_ENGINES

    def draw(self, context):
        layout = self.layout
        scene_props = context.scene.collection_properties['BLENDER_CLAY']
        collection = get_collection_from_context(context)
        collection_props = collection.engine_overrides['BLENDER_CLAY']

        col = layout.column()
        col.template_override_property(collection_props, scene_props, "matcap_icon", custom_template="icon_view")
        col.template_override_property(collection_props, scene_props, "matcap_rotation")
        col.template_override_property(collection_props, scene_props, "matcap_hue")
        col.template_override_property(collection_props, scene_props, "matcap_saturation")
        col.template_override_property(collection_props, scene_props, "matcap_value")
        col.template_override_property(collection_props, scene_props, "ssao_factor_cavity")
        col.template_override_property(collection_props, scene_props, "ssao_factor_edge")
        col.template_override_property(collection_props, scene_props, "ssao_distance")
        col.template_override_property(collection_props, scene_props, "ssao_attenuation")
        col.template_override_property(collection_props, scene_props, "hair_brightness_randomness")


class COLLECTION_PT_object_mode_settings(CollectionButtonsPanel, Panel):
    bl_label = "Object Mode Settings"

    @classmethod
    def poll(cls, context):
        window = context.window
        return window and hasattr(window, 'object_mode') and (window.object_mode == 'OBJECT')

    def draw(self, context):
        layout = self.layout
        scene_props = context.scene.collection_properties['ObjectMode']
        collection = get_collection_from_context(context)
        collection_props = collection.engine_overrides['ObjectMode']

        col = layout.column()
        col.template_override_property(collection_props, scene_props, "show_wire")
        col.template_override_property(collection_props, scene_props, "show_backface_culling")


class COLLECTION_PT_edit_mode_settings(CollectionButtonsPanel, Panel):
    bl_label = "Edit Mode Settings"

    @classmethod
    def poll(cls, context):
        window = context.window
        return window and hasattr(window, 'object_mode') and (window.object_mode == 'EDIT')

    def draw(self, context):
        layout = self.layout
        scene_props = context.scene.collection_properties['EditMode']
        collection = get_collection_from_context(context)
        collection_props = collection.engine_overrides['EditMode']

        col = layout.column()
        col.template_override_property(collection_props, scene_props, "show_occlude_wire")
        col.template_override_property(collection_props, scene_props, "backwire_opacity")
        col.template_override_property(collection_props, scene_props, "face_normals_show")
        col.template_override_property(collection_props, scene_props, "vert_normals_show")
        col.template_override_property(collection_props, scene_props, "loop_normals_show")
        col.template_override_property(collection_props, scene_props, "normals_length")
        col.template_override_property(collection_props, scene_props, "show_weight")


class COLLECTION_PT_paint_weight_mode_settings(CollectionButtonsPanel, Panel):
    bl_label = "Weight Paint Mode Settings"

    @classmethod
    def poll(cls, context):
        window = context.window
        return window and hasattr(window, 'object_mode') and (window.object_mode == 'WEIGHT_PAINT')

    def draw(self, context):
        layout = self.layout
        scene_props = context.scene.collection_properties['WeightPaintMode']
        collection = get_collection_from_context(context)
        collection_props = collection.engine_overrides['WeightPaintMode']

        col = layout.column()
        col.template_override_property(collection_props, scene_props, "use_shading")
        col.template_override_property(collection_props, scene_props, "use_wire")


class COLLECTION_PT_paint_vertex_mode_settings(CollectionButtonsPanel, Panel):
    bl_label = "Vertex Paint Mode Settings"

    @classmethod
    def poll(cls, context):
        window = context.window
        return window and hasattr(window, 'object_mode') and (window.object_mode == 'VERTEX_PAINT')

    def draw(self, context):
        layout = self.layout
        scene_props = context.scene.collection_properties['VertexPaintMode']
        collection = get_collection_from_context(context)
        collection_props = collection.engine_overrides['VertexPaintMode']

        col = layout.column()
        col.template_override_property(collection_props, scene_props, "use_shading")
        col.template_override_property(collection_props, scene_props, "use_wire")


classes = (
    COLLECTION_PT_context_collection,
    COLLECTION_PT_clay_settings,
    COLLECTION_PT_object_mode_settings,
    COLLECTION_PT_edit_mode_settings,
    COLLECTION_PT_paint_weight_mode_settings,
    COLLECTION_PT_paint_vertex_mode_settings,
)

if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
