/*
 * MyPIDControllerService.cpp
 *
 *  Created on: 30 dic 2015
 *      Author: andrea
 */

#include <dbus/MyPIDControllerServiceWrap.h>
#include <iostream>
#include <gio/gio.h>

MyPIDControllerServiceWrap::MyPIDControllerServiceWrap() {
}

MyPIDControllerServiceWrap::~MyPIDControllerServiceWrap() {
 //TODO: devo rilasciare qualche risorsa?
}
bool MyPIDControllerServiceWrap::initialize() {
	cout << "Initialize MyPIDControllerService" << endl;
	GMainLoop *loop;
	 loop = g_main_loop_new (NULL, FALSE);
	 int busID = g_bus_own_name (G_BUS_TYPE_SESSION,
	 "org.mydrone.MyPIDControllerService",
	 G_BUS_NAME_OWNER_FLAGS_NONE,
	 NULL,
	 MyPIDControllerServiceWrap::on_name_acquired,
	 NULL,
	 NULL,
	 NULL);
	cout << "Running loop MyPIDControllerService: "  << busID << endl;
	 g_main_loop_run (loop);
	cout << "return from loop MyPIDControllerService: "  << busID << endl;
	return true;
}

// static
void MyPIDControllerServiceWrap::on_name_acquired(GDBusConnection *connection, const gchar *name, gpointer user_data) {
	cout << "Name Acquired: org.mydrone.MyPIDControllerService" << endl;
//	MyPIDControllerService* skeleton;
	skeleton = my_pidcontroller_service_skeleton_new();
	/* Tolto. Verrà riaggiunto in seguito
	g_signal_connect (skeleton,
	 "handle-set-pid",
	 G_CALLBACK (MyPIDControllerServiceWrap::on_handle_set_pid),
	 NULL);
	cout << "Signal Connected: org.mydrone.MyPIDControllerService" << endl;
	*/

	g_dbus_interface_skeleton_export (G_DBUS_INTERFACE_SKELETON (skeleton),
	 connection,
	"/org/mydrone/MyPIDControllerService",
	NULL);
	cout << "Interface Exported: org.mydrone.MyPIDControllerService" << endl;
}

// static
/*
gboolean MyPIDControllerServiceWrap::on_handle_set_pid (MyPIDControllerService *skeleton,
	 GDBusMethodInvocation *invocation,
	 guint8 PID,
	 guint8 YPR,
	 gdouble value,
	 gpointer user_data) {

	cout << "RECEIVED: " << std::to_string(PID) << ", " << std::to_string(YPR) << ", " << std::to_string(value) << endl;

		my_pidcontroller_service_complete_set_pid(skeleton, invocation);
	return true;
}
*/
MyPIDControllerService* MyPIDControllerServiceWrap::skeleton = nullptr;

void MyPIDControllerServiceWrap::emitStateChangedSignal(MyPIDState pidState) {
	if(skeleton != nullptr){
/*
		GVariantBuilder builder0;
		g_variant_builder_init(&builder0, G_VARIANT_TYPE_TUPLE);
		g_variant_builder_add(&builder0, "u", (guint32)pidState.timestampMillis);

		GVariantBuilder builderYaw;
		g_variant_builder_init(&builderYaw, G_VARIANT_TYPE_TUPLE);
		g_variant_builder_add(&builderYaw, "d", (gdouble)pidState.yawSample.current_value);
		g_variant_builder_add(&builderYaw, "d", (gdouble)pidState.yawSample.target_value);
		g_variant_builder_add(&builderYaw, "d", (gdouble)pidState.yawSample.error_);
		g_variant_builder_add(&builderYaw, "d", (gdouble)pidState.yawSample.error_i);
		g_variant_builder_add(&builderYaw, "d", (gdouble)pidState.yawSample.error_d);
		g_variant_builder_add(&builder0, "(ddddd)", g_variant_builder_end(&builderYaw));

		GVariantBuilder builderPitch;
		g_variant_builder_init(&builderPitch, G_VARIANT_TYPE_TUPLE);
		g_variant_builder_add(&builderPitch, "d", (gdouble)pidState.pitchSample.current_value);
		g_variant_builder_add(&builderPitch, "d", (gdouble)pidState.pitchSample.target_value);
		g_variant_builder_add(&builderPitch, "d", (gdouble)pidState.pitchSample.error_);
		g_variant_builder_add(&builderPitch, "d", (gdouble)pidState.pitchSample.error_i);
		g_variant_builder_add(&builderPitch, "d", (gdouble)pidState.pitchSample.error_d);
		g_variant_builder_add(&builder0, "(ddddd)", g_variant_builder_end(&builderPitch));

		GVariantBuilder builderRoll;
		g_variant_builder_init(&builderRoll, G_VARIANT_TYPE_TUPLE);
		g_variant_builder_add(&builderRoll, "d", (gdouble)pidState.rollSample.current_value);
		g_variant_builder_add(&builderRoll, "d", (gdouble)pidState.rollSample.target_value);
		g_variant_builder_add(&builderRoll, "d", (gdouble)pidState.rollSample.error_);
		g_variant_builder_add(&builderRoll, "d", (gdouble)pidState.rollSample.error_i);
		g_variant_builder_add(&builderRoll, "d", (gdouble)pidState.rollSample.error_d);
		g_variant_builder_add(&builder0, "(ddddd)", g_variant_builder_end(&builderRoll));

		my_pidcontroller_service_emit_state_changed(skeleton, g_variant_builder_end(&builder0));
*/

		GVariant* gavriant_pointer = g_variant_new_from_data(
				(const GVariantType*)"(u(ddddd)(ddddd)(ddddd))" ,
//				(guint32)pidState.timestampMillis,
				&pidState ,
				sizeof(MyPIDState) , true , NULL , NULL);

		my_pidcontroller_service_emit_state_changed(skeleton, gavriant_pointer);

	}
}
