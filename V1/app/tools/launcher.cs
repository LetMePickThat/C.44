// LAUNCHER -- what the two .exe at the root of V1/ are made of (see tools/gen_launchers.py). An .exe carries its icon inside
// itself, which a .lnk cannot do: a shortcut's icon path is absolute, so it only holds on the machine that wrote it.
// Starts app\<name of this .exe>.bat, wherever the folder is; any file dropped on the .exe is passed on.
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Windows.Forms;

static class Launcher
{
    [STAThread]
    static int Main(string[] args)
    {
        string exe = Process.GetCurrentProcess().MainModule.FileName;
        string app = Path.Combine(Path.GetDirectoryName(exe), "app");
        string name = Path.GetFileNameWithoutExtension(exe);
        string bat = Path.Combine(app, name + ".bat");
        if (!File.Exists(bat))
        {
            MessageBox.Show("Cannot find:\n" + bat + "\n\nThis file must stay beside the app folder.", name,
                            MessageBoxButtons.OK, MessageBoxIcon.Error);
            return 1;
        }
        var start = new ProcessStartInfo(bat, string.Join(" ", args.Select(a => "\"" + a + "\"")));
        start.WorkingDirectory = app;
        start.UseShellExecute = true;
        Process.Start(start);
        return 0;
    }
}
