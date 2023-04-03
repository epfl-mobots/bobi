
/*!
 * This class is based on the DBusInterface from aseba/examples/clients
 * (https://github.com/aseba-community/aseba)
 */

#ifndef BOBI_DBUSINTERFACE_H
#define BOBI_DBUSINTERFACE_H

#include <QtDBus/QtDBus>
#include <functional>

Q_DECLARE_METATYPE(QList<qint16>);

typedef QList<qint16> Values;

namespace bobi {
    namespace dbus {
        typedef std::function<void(const Values&)> EventCallback;

        class DBusInterface : public QObject {
            Q_OBJECT

        public:
            DBusInterface();

            static QVariant value_to_variant(const Values& value);
            static Values dBus_msg_to_values(const QDBusMessage& dbmess, int index);
            static std::string to_string(const Values& v);

            bool check_connection();
            void display_node_list() const;

            void load_script(const QString& script);
            Values get_variable(const QString& node, const QString& variable);
            void set_variable(const QString& node, const QString& variable, const Values& value);
            void connect_event(const QString& eventName, EventCallback callback);

            void send_event(uint16_t eventID, const Values& value);
            void send_event_name(const QString& eventName, const Values& value);

        protected:
            QDBusConnection _dbus;
            std::map<QString, EventCallback> _callbacks;
            QDBusInterface _dbus_main_intf;

            QList<QString> _node_list;

            QDBusInterface* _event_filter_intf;

        public slots:
            void dispatch_event(const QDBusMessage& message);
        };

    } // namespace dbus
} // namespace bobi

#endif // DBUSINTERFACE_H
