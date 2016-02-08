/*
 * MyPIDControllerService.h
 *
 *  Created on: 30 dic 2015
 *      Author: andrea
 */

#ifndef DBUS_MYPIDCONTROLLERSERVICEWRAP_H_
#define DBUS_MYPIDCONTROLLERSERVICEWRAP_H_
#include <stdint.h>
#include <gio/gio.h>
#include <dbus/MyPIDControllerService-generated.h>
#include "MyPIDState.h"

using namespace std;
class MyPIDControllerServiceWrap {
public:
	MyPIDControllerServiceWrap();
	virtual ~MyPIDControllerServiceWrap();
	static bool initialize();
	static void emitStateChangedSignal(MyPIDState myPIDState);
protected:
	static void on_name_acquired(GDBusConnection *connection, const gchar *name, gpointer user_data);
	static gboolean on_handle_set_pid(MyPIDControllerService *skeleton,
	 GDBusMethodInvocation *invocation,
	 guint8 PID,
	 guint8 YPR,
	 gdouble value,
	 gpointer user_data);
private:
	static MyPIDControllerService *skeleton;

};
#endif /* DBUS_MYPIDCONTROLLERSERVICEWRAP_H_ */
