#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <string>

namespace gazebo
{
    class ColorChangerPlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
        {
            this->model = _model;

            if (!rclcpp::ok())
                rclcpp::init(0, nullptr);

            this->node = std::make_shared<rclcpp::Node>("color_changer_node");

            std::string topic = "/" + this->model->GetName() + "/set_color";
            this->sub = this->node->create_subscription<std_msgs::msg::String>(
                topic, 10,
                std::bind(&ColorChangerPlugin::ColorCallback, this, std::placeholders::_1));

            this->rclcpp_thread = std::thread([this]()
                                              { rclcpp::spin(this->node); });

            this->gz_node = transport::NodePtr(new transport::Node());
            this->gz_node->Init(model->GetWorld()->Name());

            this->vis_pub = this->gz_node->Advertise<msgs::Visual>("~/visual");

            std::cout << "[Plugin] Plugin carregado para modelo: " << this->model->GetName() << std::endl;
        }

        void ColorCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            std::string color = msg->data;
            float r = 0.0, g = 0.0, b = 0.0;
            
            if (color == "Normal")                  { r = 1.0; g = 1.0; b = 1.0; } // White
            else if (color == "Patrulha")           { r = 0.0; g = 0.0; b = 1.0; } // Blue
            else if (color == "Infetado")           { r = 1.0; g = 0.0; b = 0.0; } // Red
            else if (color == "Quarentena")         { r = 1.0; g = 0.5; b = 0.0; } // Orange
            else if (color == "Passageiro")         { r = 0.0; g = 1.0; b = 0.0; } // Green
            else if (color == "Passageiro/Quarentena") { r = 0.0; g = 1.0; b = 1.0; } // Cyan
            else if (color == "Patrulha/Quarentena")  { r = 1.0; g = 0.0; b = 1.0; } // Magenta
            else if (color == "Changing")  { r = 0.0; g = 0.0; b = 0.0; } // Blacky
            else
            {
                std::cerr << "[Plugin] Cor desconhecida: " << color << std::endl;
                return;
            }

            for (auto link : this->model->GetLinks())
            {
                std::string visual_name = this->model->GetName() + "::" + link->GetName() + "::visual";

                msgs::Visual visual_msg;
                visual_msg.set_name(visual_name);
                visual_msg.set_parent_name(link->GetScopedName());

                auto mat = visual_msg.mutable_material();
                mat->clear_script();

                mat->mutable_ambient()->set_r(r);
                mat->mutable_ambient()->set_g(g);
                mat->mutable_ambient()->set_b(b);
                mat->mutable_ambient()->set_a(1.0);

                mat->mutable_diffuse()->set_r(r);
                mat->mutable_diffuse()->set_g(g);
                mat->mutable_diffuse()->set_b(b);
                mat->mutable_diffuse()->set_a(1.0);

                this->vis_pub->Publish(visual_msg);
                std::cout << "[Plugin] Cor alterada para " << color << " no visual: " << visual_name << std::endl;
            }
        }

    private:
        physics::ModelPtr model;
        transport::NodePtr gz_node;
        transport::PublisherPtr vis_pub;

        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
        std::thread rclcpp_thread;
    };

    GZ_REGISTER_MODEL_PLUGIN(ColorChangerPlugin)
}
