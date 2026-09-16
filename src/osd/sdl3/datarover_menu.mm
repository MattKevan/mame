// license:BSD-3-Clause
// copyright-holders:MAMEdev Team
//============================================================
//
//  datarover_menu.mm - native macOS menubar for the SDL3 OSD
//
//  Installed from sdl_osd_interface::init on macOS builds.  The SDL3
//  backend otherwise leaves the Cocoa menubar empty, so the process
//  falls back to a bare application menu with no useful entries.
//  This module builds a small DataRover-branded menubar whose items
//  call the same OSD/emu entry points as the keyboard shortcuts and
//  the on-screen (Tab) Video Options rows.
//
//============================================================

#if defined(SDLMAME_MACOSX)

#import <Cocoa/Cocoa.h>

#include "emu.h"
#include "render.h"
#include "ui/uimain.h"
#include "modules/lib/osdobj_common.h"
#include "modules/osdwindow.h"
#include "window.h"

// Forward-declared by the SDL3 OSD; the init hook calls it on macOS.
extern "C" void datarover_install_menu(void);

// Nibless action target.  Held by the menubar for process lifetime.
@interface DataRoverMenuHandler : NSObject
- (void)resetMachine:(id)sender;
- (void)freshBoot:(id)sender;
- (void)selectLCD:(id)sender;
- (void)selectSerial:(id)sender;
- (void)selectBoth:(id)sender;
- (void)cycleZoom:(id)sender;
- (void)toggleFullscreen:(id)sender;
- (void)toggleMameUI:(id)sender;
@end

namespace {

running_machine *current_machine()
{
	if (osd_common_t::window_list().empty() || (osd_common_t::window_list().front() == nullptr))
		return nullptr;
	return &osd_common_t::window_list().front()->machine();
}

render_target *first_live_target(running_machine &machine)
{
	for (render_target *target = machine.render().first_target(); target != nullptr; target = target->next())
	{
		if (!target->hidden())
			return target;
	}
	return nullptr;
}

sdl_window_info *first_live_window(running_machine &machine)
{
	for (auto &window : osd_common_t::window_list())
	{
		auto *sdlwin = dynamic_cast<sdl_window_info *>(window.get());
		if ((sdlwin != nullptr) && (sdlwin->target() != nullptr) && !sdlwin->target()->hidden())
			return sdlwin;
	}
	return nullptr;
}

} // anonymous namespace

@implementation DataRoverMenuHandler

- (void)resetMachine:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
		machine->schedule_hard_reset();
}

- (void)freshBoot:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
		machine->schedule_hard_reset();
}

- (void)selectLCD:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
	{
		if (render_target *target = first_live_target(*machine))
			target->set_view(0);
	}
}

- (void)selectSerial:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
	{
		if (render_target *target = first_live_target(*machine))
			target->set_view(1);
	}
}

- (void)selectBoth:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
	{
		if (render_target *target = first_live_target(*machine))
			target->set_view(2);
	}
}

- (void)cycleZoom:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
		if (sdl_window_info *osdwin = first_live_window(*machine))
		{
			// Mirror IPT_OSD_6/IPT_OSD_7 (window prescale steppers):
			// wrap from max back to 1x so a single item cycles.
			if (osdwin->prescale() >= 20)
			{
				while (osdwin->prescale() > 1)
					osdwin->modify_prescale(-1);
			}
			else
			{
				osdwin->modify_prescale(1);
			}
		}
}

- (void)toggleFullscreen:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
		if (sdl_window_info *osdwin = first_live_window(*machine))
		{
			for (auto &curwin : osd_common_t::window_list())
				curwin->renderer_reset();
			osdwin->toggle_full_screen();
		}
}

- (void)toggleMameUI:(id)sender
{
	(void)sender;
	if (running_machine *machine = current_machine())
	{
		ui_manager &ui = machine->ui();
		if (ui.is_menu_active())
			ui.menu_reset();
		// else: no documented base-class entry point opens the MAME menu;
		// mame_ui_manager::show_menu() lives in frontend headers outside the
		// OSD include path, so showing is intentionally left unwired.
	}
}

@end

namespace {

DataRoverMenuHandler *menu_handler()
{
	static DataRoverMenuHandler *handler = [[DataRoverMenuHandler alloc] init];
	return handler;
}

NSMenuItem *menu_entry(NSString *title, NSString *key, SEL action)
{
	NSMenuItem *item = [[[NSMenuItem alloc] initWithTitle:title action:action keyEquivalent:key] autorelease];
	[item setTarget:menu_handler()];
	return item;
}

void install_submenu(NSMenu *bar, NSString *title, NSMenu *submenu)
{
	NSMenuItem *holder = [[[NSMenuItem alloc] initWithTitle:title action:nil keyEquivalent:@""] autorelease];
	[holder setSubmenu:submenu];
	[bar addItem:holder];
}

} // anonymous namespace

extern "C" void datarover_install_menu(void)
{
	@autoreleasepool
	{
		NSMenu *bar = [[[NSMenu alloc] initWithTitle:@"DataRover"] autorelease];

		NSMenu *appMenu = [[[NSMenu alloc] initWithTitle:@"DataRover"] autorelease];
		NSMenuItem *about = menu_entry(@"About DataRover 840", @"", @selector(orderFrontStandardAboutPanel:));
		[about setTarget:nil];
		[appMenu addItem:about];
		[appMenu addItem:[NSMenuItem separatorItem]];
		NSMenuItem *quit = menu_entry(@"Quit DataRover 840", @"q", @selector(terminate:));
		[quit setTarget:nil];
		[appMenu addItem:quit];
		install_submenu(bar, @"DataRover", appMenu);

		NSMenu *fileMenu = [[[NSMenu alloc] initWithTitle:@"File"] autorelease];
		[fileMenu addItem:menu_entry(@"Reset Machine", @"r", @selector(resetMachine:))];
		[fileMenu addItem:menu_entry(@"Fresh Boot", @"", @selector(freshBoot:))];
		install_submenu(bar, @"File", fileMenu);

		NSMenu *viewMenu = [[[NSMenu alloc] initWithTitle:@"View"] autorelease];
		[viewMenu addItem:menu_entry(@"LCD", @"1", @selector(selectLCD:))];
		[viewMenu addItem:menu_entry(@"Serial Terminal", @"2", @selector(selectSerial:))];
		[viewMenu addItem:menu_entry(@"LCD and Serial", @"3", @selector(selectBoth:))];
		[viewMenu addItem:[NSMenuItem separatorItem]];
		[viewMenu addItem:menu_entry(@"Zoom Cycle", @"0", @selector(cycleZoom:))];
		[viewMenu addItem:menu_entry(@"Fullscreen", @"f", @selector(toggleFullscreen:))];
		install_submenu(bar, @"View", viewMenu);

		NSMenu *windowMenu = [[[NSMenu alloc] initWithTitle:@"Window"] autorelease];
		[windowMenu addItem:menu_entry(@"Toggle MAME UI", @"", @selector(toggleMameUI:))];
		NSMenuItem *minimize = [[[NSMenuItem alloc] initWithTitle:@"Minimize" action:@selector(performMiniaturize:) keyEquivalent:@"m"] autorelease];
		[minimize setTarget:nil];
		[windowMenu addItem:minimize];
		install_submenu(bar, @"Window", windowMenu);

		[NSApp setMainMenu:bar];
	}
}

#endif // defined(SDLMAME_MACOSX)
