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
- (void)installPackage:(id)sender;
- (void)closeWindow:(id)sender;
- (void)selectLCD:(id)sender;
- (void)selectSerial:(id)sender;
- (void)selectBoth:(id)sender;
- (void)setActualSize:(id)sender;
- (void)setDoubleSize:(id)sender;
- (void)toggleMameUI:(id)sender;
- (void)pressPower:(id)sender;
- (void)pressOption:(id)sender;
- (void)releasePendingButton:(id)sender;
@end

namespace {

// Momentary button press state: the Device-menu release is deferred
// ~150 ms so the per-frame ioport scan observes the pressed edge
// before the release edge.
static ioport_field *s_pending_release = nullptr;


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
	{
		// Fresh Boot = cold start: reset all NVRAM devices to defaults
		// in memory (device_nvram_interface::nvram_reset, the same call
		// running_machine::nvram_load uses on a file miss) so the
		// machine-run exit phase persists defaults instead of the live
		// heap, delete the on-disk files so the next process starts
		// clean, then hard-reset.
		for (device_nvram_interface &nvram : nvram_interface_enumerator(machine->root_device()))
			nvram.nvram_reset();
		NSString *nvdir = [NSString stringWithUTF8String:machine->options().nvram_directory()];
		NSString *base = [NSString stringWithUTF8String:machine->basename().c_str()];
		if ((nvdir != nil) && (base != nil))
		{
			NSString *sysdir = [nvdir stringByAppendingPathComponent:base];
			NSFileManager *fm = [NSFileManager defaultManager];
			NSArray<NSString *> *files = [fm contentsOfDirectoryAtPath:sysdir error:nil];
			for (NSString *file in files)
				[fm removeItemAtPath:[sysdir stringByAppendingPathComponent:file] error:nil];
		}
		machine->schedule_hard_reset();
	}
}

- (void)installPackage:(id)sender
{
	(void)sender;
	running_machine *machine = current_machine();
	if (!machine)
		return;
	// PTY discovery: env var primary, run-file fallback.  The launcher
	// exports MAGIC_CAP_EMULATOR_ROOT (known pre-exec) but cannot export
	// DATAROVER_PCLINK_PTY — the slave path only exists after the emulator
	// starts, and exec freezes the environment — so a disowned scraper
	// publishes the ":rs2321:pty PTY: <path>" announcement to
	// ~/Library/Application Support/DataRover/run/pclink-pty.  Walking
	// machine.root_device() for the "pty" device plus slave_name() is
	// fragile from ObjC++, so env-then-file is the injection point.
	const char *pty = getenv("DATAROVER_PCLINK_PTY");
	NSString *ptyPath = nil;
	if (pty && *pty)
		ptyPath = [NSString stringWithUTF8String:pty];
	if (!ptyPath)
	{
		NSString *runFile = [NSHomeDirectory() stringByAppendingPathComponent:
			@"Library/Application Support/DataRover/run/pclink-pty"];
		NSError *readError = nil;
		NSString *contents = [NSString stringWithContentsOfFile:runFile
			encoding:NSUTF8StringEncoding error:&readError];
		if (!readError)
		{
			ptyPath = [contents stringByTrimmingCharactersInSet:
				[NSCharacterSet whitespaceAndNewlineCharacterSet]];
			if ([ptyPath length] == 0)
				ptyPath = nil;
		}
	}
	if (!ptyPath)
	{
		machine->ui().popup_time(5, "PCLink serial port not available");
		return;
	}
	NSOpenPanel *panel = [NSOpenPanel openPanel];
	[panel setCanChooseFiles:YES];
	[panel setCanChooseDirectories:NO];
	[panel setAllowsMultipleSelection:NO];
	[panel setAllowedFileTypes:@[@"pkg"]];
	NSString *pkgdir = [NSHomeDirectory() stringByAppendingPathComponent:
		@"Library/Application Support/DataRover/packages"];
	[[NSFileManager defaultManager] createDirectoryAtPath:pkgdir
		withIntermediateDirectories:YES attributes:nil error:nil];
	[panel setDirectoryURL:[NSURL fileURLWithPath:pkgdir]];
	if ([panel runModal] != NSModalResponseOK)
		return;
	NSString *path = [[panel URL] path];
	if (!path)
		return;
	// pclink_send.py location: repo tools/ path.  The emulator checkout
	// (magic-cap-emulator) ships tools/pclink_send.py next to the MAME
	// fork; the .app bundle does not embed Resources for helper scripts
	// yet, so resolve relative to $MAGIC_CAP_EMULATOR_ROOT, falling back
	// to the fork-adjacent ../magic-cap-emulator checkout layout.
	const char *root = getenv("MAGIC_CAP_EMULATOR_ROOT");
	NSString *script = nil;
	if (root && *root)
		script = [NSString stringWithFormat:@"%s/tools/pclink_send.py", root];
	else
		script = [[[NSBundle mainBundle] bundlePath]
			stringByAppendingPathComponent:@"../../../magic-cap-emulator/tools/pclink_send.py"];
	script = [script stringByStandardizingPath];
	if (![[NSFileManager defaultManager] isExecutableFileAtPath:@"/usr/bin/python3"] ||
		![[NSFileManager defaultManager] fileExistsAtPath:script])
	{
		machine->ui().popup_time(5, "pclink_send.py not found");
		return;
	}
	NSTask *task = [[[NSTask alloc] init] autorelease];
	[task setLaunchPath:@"/usr/bin/python3"];
	[task setArguments:@[script, @"--pty", ptyPath,
		@"--package", path]];
	@try
	{
		[task launch];
	}
	@catch (NSException *e)
	{
		(void)e;
		machine->ui().popup_time(5, "Could not start package install");
	}
}

- (void)closeWindow:(id)sender
 {
 	(void)sender;
 	[[NSApp keyWindow] performClose:sender];
 }

// Momentary button press: drive the same IPT_OTHER port the keyboard
// binding feeds (root_device().ioport(tag)->field(mask)), via the
// ioport_field::set_value/clear_value injection point used by the
// on-screen clickable layout views (render.cpp).  The frame_update
// changed-callback then fires power_changed/option_changed exactly as a
// physical key press would.
- (void)releasePendingButton:(id)sender
{
	(void)sender;
	if (s_pending_release)
	{
		s_pending_release->clear_value();
		s_pending_release = nullptr;
	}
}

- (void)pulsePort:(const char *)tag
{
	if (running_machine *machine = current_machine())
	{
		[self releasePendingButton:nil];
		ioport_port *port = machine->root_device().ioport(tag);
		if (port)
		{
			if (ioport_field *field = port->field(0x01))
			{
				field->set_value(1);
				s_pending_release = field;
				[self performSelector:@selector(releasePendingButton:)
					withObject:nil afterDelay:0.15];
			}
		}
	}
}
- (void)pressPower:(id)sender
{
	(void)sender;
	[self pulsePort:"POWER_BUTTON"];
}

- (void)pressOption:(id)sender
{
	(void)sender;
	[self pulsePort:"OPTION_BUTTON"];
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

- (void)setWindowScale:(int)scale
{
	if (running_machine *machine = current_machine())
		if (sdl_window_info *osdwin = first_live_window(*machine))
		{
			if (osdwin->fullscreen())
			{
				for (auto &curwin : osd_common_t::window_list())
					curwin->renderer_reset();
				osdwin->toggle_full_screen();
			}
			int32_t minwidth, minheight;
			osdwin->target()->compute_minimum_size(minwidth, minheight);
			while (osdwin->prescale() > scale)
				osdwin->modify_prescale(-1);
			while (osdwin->prescale() < scale)
				osdwin->modify_prescale(1);
			osdwin->resize(minwidth * scale, minheight * scale);
		}
}

- (void)setActualSize:(id)sender
{
	(void)sender;
	[self setWindowScale:1];
}

- (void)setDoubleSize:(id)sender
{
	(void)sender;
	[self setWindowScale:2];
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
		else
			ui.show_main_menu();
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
		install_submenu(bar, @"DataRover", appMenu);

	NSMenu *fileMenu = [[[NSMenu alloc] initWithTitle:@"File"] autorelease];
	[fileMenu addItem:menu_entry(@"Reset Machine", @"r", @selector(resetMachine:))];
	[fileMenu addItem:menu_entry(@"Fresh Boot", @"", @selector(freshBoot:))];
	[fileMenu addItem:menu_entry(@"Install Package…", @"i", @selector(installPackage:))];
	[fileMenu addItem:menu_entry(@"Close", @"w", @selector(closeWindow:))];
	install_submenu(bar, @"File", fileMenu);

		NSMenu *viewMenu = [[[NSMenu alloc] initWithTitle:@"View"] autorelease];
		[viewMenu addItem:menu_entry(@"LCD", @"1", @selector(selectLCD:))];
		[viewMenu addItem:menu_entry(@"Serial Terminal", @"2", @selector(selectSerial:))];
		[viewMenu addItem:menu_entry(@"LCD and Serial", @"3", @selector(selectBoth:))];
		[viewMenu addItem:[NSMenuItem separatorItem]];
		[viewMenu addItem:menu_entry(@"Actual Size", @"0", @selector(setActualSize:))];
		[viewMenu addItem:menu_entry(@"2x", @"", @selector(setDoubleSize:))];
		[viewMenu addItem:menu_entry(@"Fullscreen", @"f", @selector(toggleFullscreen:))];
		install_submenu(bar, @"View", viewMenu);

		NSMenu *windowMenu = [[[NSMenu alloc] initWithTitle:@"Window"] autorelease];
		[windowMenu addItem:menu_entry(@"Toggle MAME UI", @"", @selector(toggleMameUI:))];
		NSMenuItem *minimize = [[[NSMenuItem alloc] initWithTitle:@"Minimize" action:@selector(performMiniaturize:) keyEquivalent:@"m"] autorelease];
		[minimize setTarget:nil];
		[windowMenu addItem:minimize];
	install_submenu(bar, @"Window", windowMenu);

	NSMenu *deviceMenu = [[[NSMenu alloc] initWithTitle:@"Device"] autorelease];
	[deviceMenu addItem:menu_entry(@"Power", @"p", @selector(pressPower:))];
	[deviceMenu addItem:menu_entry(@"Option Button", @"", @selector(pressOption:))];
	install_submenu(bar, @"Device", deviceMenu);

	[NSApp setMainMenu:bar];
	}
}

#endif // defined(SDLMAME_MACOSX)
