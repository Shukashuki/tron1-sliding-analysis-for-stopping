param([Parameter(Mandatory=$true)][string]$Output)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public class IsaacWindowCapture {
    [StructLayout(LayoutKind.Sequential)] public struct Rect { public int Left, Top, Right, Bottom; }
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hwnd, out Rect rect);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hwnd, IntPtr hdc, uint flags);
}
'@
$process = @(Get-Process kit | Where-Object { $_.MainWindowTitle -like 'Isaac Sim*' })
if ($process.Count -ne 1) { throw 'Expected exactly one visible Isaac Sim window.' }
$rect = New-Object IsaacWindowCapture+Rect
[void][IsaacWindowCapture]::GetWindowRect($process[0].MainWindowHandle, [ref]$rect)
$bitmap = New-Object System.Drawing.Bitmap(($rect.Right-$rect.Left), ($rect.Bottom-$rect.Top))
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
$handle = $graphics.GetHdc()
try {
    if (-not [IsaacWindowCapture]::PrintWindow($process[0].MainWindowHandle, $handle, 2)) { throw 'PrintWindow failed.' }
} finally { $graphics.ReleaseHdc($handle) }
try { $bitmap.Save($Output, [System.Drawing.Imaging.ImageFormat]::Png) }
finally { $graphics.Dispose(); $bitmap.Dispose() }
