/**
 * @file gear_switch_video_layer.hpp
 * @brief Center panel showing rearleft + rearright side by side as permanent rear view.
 * @copyright 2024 TUMFTM
**/

#pragma once

#include <algorithm>

#include "glad/glad.h"

#include "tod_gl/layers/docking_scene_layer.hpp"
#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/renderer/renderer.hpp"
#include "tod_gl/renderer/renderer_command.hpp"
#include "tod_gl/ros_interface/subscribing_components/image_component.hpp"
#include "tod_gl/systems/shader_system.hpp"

namespace tod_visual {

class GearSwitchVideoLayer : public tod_gl::DockingSceneLayer {
  public:
    GearSwitchVideoLayer(std::shared_ptr<tod_gl::RosInterface> ros,
                         std::shared_ptr<tod_gl::Scene> scene,
                         ImGuiDir split_dir, std::string name)
        : tod_gl::DockingSceneLayer(ros, scene, split_dir) {
        _name = name;
    }

    ~GearSwitchVideoLayer() {
        tod_gl::Renderer::delete_texture(left_tex_);
        tod_gl::Renderer::delete_texture(right_tex_);
    }

    // false → not hidden by show_videos=false in DIRECT mode
    bool is_video_layer() const override { return false; }

    void on_attach() override {
        shader_ = tod_gl::ShaderSystem::create_shader_program(
            (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video_layer.vert").c_str(),
            (tod_gl::RosInterface::get_package_path() + "/resources/shaders/video_rgb.frag").c_str());

        left_tex_  = tod_gl::Texture(tex_w_, tex_h_, "rear_left_tex",  GL_TEXTURE_2D, GL_RGB8, GL_RGB);
        right_tex_ = tod_gl::Texture(tex_w_, tex_h_, "rear_right_tex", GL_TEXTURE_2D, GL_RGB8, GL_RGB);

        tod_gl::Renderer::generate_texture(left_tex_,  nullptr, shader_, k_left_unit);
        tod_gl::Renderer::generate_texture(right_tex_, nullptr, shader_, k_right_unit);

        left_front_  = tod_gl::Buffer(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW);
        left_back_   = tod_gl::Buffer(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW);
        right_front_ = tod_gl::Buffer(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW);
        right_back_  = tod_gl::Buffer(GL_PIXEL_UNPACK_BUFFER, GL_DYNAMIC_DRAW);

        tod_gl::Renderer::create_buffer(left_front_,  nullptr, tex_w_ * tex_h_ * 3);
        tod_gl::Renderer::create_buffer(left_back_,   nullptr, tex_w_ * tex_h_ * 3);
        tod_gl::Renderer::create_buffer(right_front_, nullptr, tex_w_ * tex_h_ * 3);
        tod_gl::Renderer::create_buffer(right_back_,  nullptr, tex_w_ * tex_h_ * 3);
    }

    void on_event(tod_gl::Event& e) override {
        tod_gl::ImGuiSceneLayer::on_event(e);
    }

    void on_update(float ts) override {
        tod_gl::Entity sm = _active_scene->find_entity_with_tag("SubscriptionManager");

        if (sm.has_component<tod_gl::ImageComponentRearLeft>()) {
            const auto& img = sm.get_component<tod_gl::ImageComponentRearLeft>().image;
            if (!img.data.empty())
                upload(left_tex_, k_left_unit, left_front_, left_back_, left_flip_, img);
        }
        if (sm.has_component<tod_gl::ImageComponentRearRight>()) {
            const auto& img = sm.get_component<tod_gl::ImageComponentRearRight>().image;
            if (!img.data.empty())
                upload(right_tex_, k_right_unit, right_front_, right_back_, right_flip_, img);
        }
    }

    void on_im_gui_render() override {
        ImGui::SameLine();
        ImGui::Begin(_dock_space_window_name.c_str());
        ImGui::SameLine();
        ImGui::Begin(_name.c_str());

        if (ImGui::IsWindowCollapsed()) {
            ImGui::End();
            ImGui::End();
            return;
        }

        render_side_by_side();

        ImGui::End();
        ImGui::End();
    }

  private:
    static constexpr GLuint k_left_unit  = 14;
    static constexpr GLuint k_right_unit = 15;

    unsigned int shader_ = 0;

    tod_gl::Texture left_tex_;
    tod_gl::Texture right_tex_;
    tod_gl::Buffer  left_front_,  left_back_;
    tod_gl::Buffer  right_front_, right_back_;
    bool left_flip_  = false;
    bool right_flip_ = false;

    float tex_w_ = 640.0f;
    float tex_h_ = 360.0f;

    void upload(tod_gl::Texture& tex, GLuint unit,
                tod_gl::Buffer& buf_a, tod_gl::Buffer& buf_b, bool& flip,
                const sensor_msgs::msg::Image& img) {
        if ((float)img.width != tex.width || (float)img.height != tex.height) {
            tex.width  = img.width;
            tex.height = img.height;
            tod_gl::Renderer::delete_texture(tex);
            tod_gl::Renderer::generate_texture(tex, nullptr, shader_, unit);
            tod_gl::Renderer::create_buffer(buf_a, nullptr, img.width * img.height * 3);
            tod_gl::Renderer::create_buffer(buf_b, nullptr, img.width * img.height * 3);
        }
        tod_gl::ShaderSystem::use_shader_program(shader_);
        tod_gl::Buffer& buf = flip ? buf_b : buf_a;
        tod_gl::Renderer::update_texture(tex, unit, buf, 0, 0, img.width, img.height,
            const_cast<void*>(static_cast<const void*>(img.data.data())));
        tod_gl::ShaderSystem::use_shader_program(0);
        flip = !flip;
    }

    void render_side_by_side() {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float half_w = avail.x * 0.5f;

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

        if (left_tex_.id != 0) {
            tod_gl::ShaderSystem::use_shader_program(shader_);
            tod_gl::RenderCommand::ForTexture::active_and_bind(left_tex_, k_left_unit);
            ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(left_tex_.id)),
                         ImVec2(half_w, avail.y), ImVec2(1.0f, 0.0f), ImVec2(0.0f, 1.0f));
            tod_gl::RenderCommand::ForTexture::unbind(left_tex_);
            tod_gl::ShaderSystem::use_shader_program(0);
        }

        ImGui::SameLine();

        if (right_tex_.id != 0) {
            tod_gl::ShaderSystem::use_shader_program(shader_);
            tod_gl::RenderCommand::ForTexture::active_and_bind(right_tex_, k_right_unit);
            ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(right_tex_.id)),
                         ImVec2(half_w, avail.y), ImVec2(1.0f, 0.0f), ImVec2(0.0f, 1.0f));
            tod_gl::RenderCommand::ForTexture::unbind(right_tex_);
            tod_gl::ShaderSystem::use_shader_program(0);
        }

        ImGui::PopStyleVar();
    }
};

}  // namespace tod_visual
