#include <bobi_fishbot_v4_interface/dbus_interface.h>

#include <ros/ros.h>

namespace bobi {
    namespace dbus {

        DBusInterface::DBusInterface() : _dbus(QDBusConnection::sessionBus()),
                                         _callbacks({}),
                                         _dbus_main_intf("ch.epfl.mobots.Aseba", "/", "ch.epfl.mobots.AsebaNetwork", _dbus)
        {
            check_connection();

            QDBusMessage event_msg = _dbus_main_intf.call("CreateEventFilter");
            QDBusObjectPath event_path = event_msg.arguments().at(0).value<QDBusObjectPath>();
            if (event_path.path().size()) {
                _event_filter_intf = new QDBusInterface("ch.epfl.mobots.Aseba", event_path.path(), "ch.epfl.mobots.EventFilter", _dbus);
                if (!_dbus.connect("ch.epfl.mobots.Aseba",
                        event_path.path(),
                        "ch.epfl.mobots.EventFilter",
                        "Event",
                        this,
                        SLOT(dispatch_event(const QDBusMessage&)))) {
                    ROS_INFO_STREAM("Failed to connect to the event filter slot");
                }
                else {
                    ROS_INFO_STREAM("Connected to the event filter slot");
                }
            }
            else {
                ROS_INFO_STREAM("Event filter path is empty, skipping");
            }
        }

        bool DBusInterface::check_connection()
        {
            if (!QDBusConnection::sessionBus().isConnected()) {
                ROS_ERROR("Can't connect to the DBus session");
                return false;
            }

            ROS_INFO_STREAM("Connected successfully to the DBus session");

            QDBusMessage node_list_msg = _dbus_main_intf.call("GetNodesList");
            for (size_t i = 0; i < node_list_msg.arguments().size(); ++i) {
                _node_list << node_list_msg.arguments().at(i).value<QString>();
            }
            display_node_list();

            return true;
        }

        void DBusInterface::display_node_list() const
        {
            if (_node_list.size()) {
                ROS_INFO_STREAM("Nodes found on the bus:");
                for (const QString node : _node_list) {
                    ROS_INFO_STREAM(node.toStdString());
                }
            }
            else {
                ROS_INFO_STREAM("No nodes found on the bus");
            }
        }

        void DBusInterface::load_script(const QString& script)
        {
            QFile file(script);
            QFileInfo fi(script);
            QString path = fi.absoluteFilePath();
            if (!file.exists()) {
                ROS_ERROR("Failed to find .aesl file");
            }
            else {
                _dbus_main_intf.call("LoadScripts", path);
            }
        }

        Values DBusInterface::get_variable(const QString& node, const QString& variable)
        {
            return dBus_msg_to_values(_dbus_main_intf.call("GetVariable", node, variable), 0);
        }

        void DBusInterface::set_variable(const QString& node, const QString& variable, const Values& value)
        {
            _dbus_main_intf.call("SetVariable", node, variable, value_to_variant(value));
        }

        void DBusInterface::connect_event(const QString& event, EventCallback callback)
        {
            _callbacks[event] = callback;
            _event_filter_intf->call("ListenEventName", event);
        }

        QVariant DBusInterface::value_to_variant(const Values& value)
        {
            QDBusArgument qdbarg;
            qdbarg << value;
            QVariant qdbvar;
            qdbvar.setValue(qdbarg);
            return qdbvar;
        }

        Values DBusInterface::dBus_msg_to_values(const QDBusMessage& dbmess, int index)
        {
            QDBusArgument read = dbmess.arguments().at(index).value<QDBusArgument>();
            Values values;
            read >> values;
            return values;
        }

        std::string DBusInterface::to_string(const Values& v)
        {
            std::string out = "[";
            for (size_t i = 0; i < v.size(); ++i) {
                out += std::to_string(v[i]);
                if (i < (v.size() - 1)) {
                    out += ",";
                }
            }
            out += "]";
            return out;
        }

        void DBusInterface::send_event(uint16_t event_id, const Values& value)
        {
            QDBusArgument argument;
            argument << event_id;
            QVariant variant;
            variant.setValue(argument);
            _dbus_main_intf.call("SendEvent", variant, value_to_variant(value));
        }

        void DBusInterface::send_event_name(const QString& eventName, const Values& value)
        {
            _dbus_main_intf.call("SendEventName", eventName, value_to_variant(value));
        }

        void DBusInterface::dispatch_event(const QDBusMessage& message)
        {
            QString event_rcv_name = message.arguments().at(1).value<QString>();
            Values event_rcv_values = dBus_msg_to_values(message, 2);
            if (_callbacks.count(event_rcv_name) > 0) {
                _callbacks[event_rcv_name](event_rcv_values);
            }
        }

    } // namespace dbus
} // namespace bobi
