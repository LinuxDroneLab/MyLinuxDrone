/*
 * MyPIDControllerService.cpp
 *
 *  Created on: 30 dic 2015
 *      Author: andrea
 */

#include <dbus/MyPIDControllerServiceWrap.h>
#include <iostream>

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
	g_signal_connect (skeleton,
	 "handle-set-pid",
	 G_CALLBACK (MyPIDControllerServiceWrap::on_handle_set_pid),
	 NULL);
	cout << "Signal Connected: org.mydrone.MyPIDControllerService" << endl;

	g_dbus_interface_skeleton_export (G_DBUS_INTERFACE_SKELETON (skeleton),
	 connection,
	"/org/mydrone/MyPIDControllerService",
	NULL);
	cout << "Interface Exported: org.mydrone.MyPIDControllerService" << endl;
}

// static
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

MyPIDControllerService* MyPIDControllerServiceWrap::skeleton = nullptr;

void MyPIDControllerServiceWrap::emitStateChangedSignal(long timestampMillis, float yawCurr,float pitchCurr,float rollCurr, float yawTrg,float pitchTrg,float rollTrg,float eRoll, float eIRoll, float eDRoll, float ePitch, float eIPitch, float eDPitch, float eYaw, float eIYaw, float eDYaw) {
	if(skeleton != nullptr){
		my_pidcontroller_service_emit_state_changed(skeleton, timestampMillis, yawCurr, pitchCurr, rollCurr,yawTrg, pitchTrg, rollTrg, eRoll, eIRoll, eDRoll, ePitch, eIPitch, eDPitch, eYaw, eIYaw, eDYaw);
	}
}
