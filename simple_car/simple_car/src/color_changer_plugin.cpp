#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh> // ✅ Adiciona definições completas de Model, Link etc.
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

            gz_node = transport::NodePtr(new transport::Node());
            gz_node->Init(model->GetWorld()->Name());

            vis_pub = gz_node->Advertise<msgs::Visual>("~/visual");
        }

        void ColorCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            std::string color = msg->data;
            physics::LinkPtr link = this->model->GetLink(); // assume que tem só 1 link
            if (!link)
            {
                std::cerr << "[Plugin] Link inválido." << std::endl;
                return;
            }

            std::string visual_name = link->GetName() + "::visual";

            msgs::Visual visual_msg;
            visual_msg.set_name(visual_name);
            visual_msg.set_parent_name(this->model->GetScopedName());

            auto mat = visual_msg.mutable_material();

            // Inicializar com preto e alfa 1.0
            float r = 0.0, g = 0.0, b = 0.0;
            if (color == "red")
            {
                r = 1.0;
            }
            else if (color == "green")
            {
                g = 1.0;
            }
            else if (color == "blue")
            {
                b = 1.0;
            }
            else
            {
                std::cerr << "[Plugin] Cor desconhecida: " << color << std::endl;
                return;
            }

            // Preencher campos obrigatórios para ambient e diffuse
            mat->mutable_ambient()->set_r(r);
            mat->mutable_ambient()->set_g(g);
            mat->mutable_ambient()->set_b(b);
            mat->mutable_ambient()->set_a(1.0);

            mat->mutable_diffuse()->set_r(r);
            mat->mutable_diffuse()->set_g(g);
            mat->mutable_diffuse()->set_b(b);
            mat->mutable_diffuse()->set_a(1.0);

            this->vis_pub->Publish(visual_msg);
            std::cout << "[Plugin] Cor alterada para: " << color << std::endl;
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
