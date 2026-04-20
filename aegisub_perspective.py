"""
Aegisub Perspektif Hesaplayıcı
-------------------------------
CLI kullanımı:
    python aegisub_perspective.py "x1,y1,x2,y2,x3,y3,x4,y4"
    python aegisub_perspective.py "x1,y1,x2,y2,x3,y3,x4,y4" --ratio 2.5
    python aegisub_perspective.py "x1,y1,x2,y2,x3,y3,x4,y4" --size 800x300

GUI kullanımı:
    python aegisub_perspective.py --gui
    python aegisub_perspective.py   (argümansız = GUI açılır)

Köşe sırası: sol_üst, sağ_üst, sağ_alt, sol_alt  (saat yönü)
"""

import sys, math, argparse, re
import numpy as np

#----------HESAPLAMA MOTORU----------

def parse_floats(s):
    return list(map(float, re.findall(
        r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?', s)))

def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def line_intersect(p1, p2, p3, p4):
    x1,y1=p1; x2,y2=p2; x3,y3=p3; x4,y4=p4
    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(denom) < 1e-10:
        return None
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom
    return (x1 + t*(x2-x1), y1 + t*(y2-y1))

def estimate_real_size(A, B, C, D):
    """Vanishing point + cross-ratio ile gerçek boyut tahmini."""
    vp_h = line_intersect(A, D, B, C)
    vp_v = line_intersect(A, B, D, C)
    top   = dist(A, B); bottom = dist(D, C)
    left  = dist(A, D); right  = dist(B, C)
    avg_w = (top + bottom) / 2
    avg_h = (left + right) / 2
    THRESHOLD = 1e6
    if vp_h is None or dist(vp_h, A) > THRESHOLD:
        real_w = avg_w
    else:
        d_top = dist(vp_h, A); d_bot = dist(vp_h, D)
        real_w = top * math.sqrt(d_top / max(d_bot, 1e-6)) if d_bot > 0 else avg_w
    if vp_v is None or dist(vp_v, A) > THRESHOLD:
        real_h = avg_h
    else:
        d_left = dist(vp_v, A); d_right = dist(vp_v, B)
        real_h = left * math.sqrt(d_left / max(d_right, 1e-6)) if d_right > 0 else avg_h
    return real_w, real_h

def solve_homography_dlt(src, dst):
    """cv2 olmadan 4 nokta homografisi (DLT)."""
    rows = []
    for (x,y),(xp,yp) in zip(src, dst):
        rows.append([-x,-y,-1, 0, 0, 0, x*xp, y*xp, xp])
        rows.append([ 0, 0, 0,-x,-y,-1, x*yp, y*yp, yp])
    A = np.array(rows)
    _, _, Vt = np.linalg.svd(A)
    return Vt[-1].reshape(3, 3)

def homography_from_corners(A, B, C, D, W, H):
    """cv2 varsa getPerspectiveTransform (daha kararlı), yoksa DLT."""
    src = np.float32([A, B, C, D])
    dst = np.float32([[0,0],[W,0],[W,H],[0,H]])
    try:
        import cv2
        return cv2.getPerspectiveTransform(src, dst)
    except ImportError:
        return solve_homography_dlt(src, dst)

def matrix_to_aegisub(M, A, B, C, D, W, H):
    """Homografi matrisi + köşe geometrisinden Aegisub etiketlerini çıkarır."""
    M = M / M[2, 2]
    AB = (B[0]-A[0], B[1]-A[1])
    AD = (D[0]-A[0], D[1]-A[1])
    # frz: Aegisub'da y aşağı pozitif ve frz ters yönlü
    frz  = -math.degrees(math.atan2(AB[1], AB[0]))
    fscx = (dist(A, B) / W) * 100
    fscy = (dist(A, D) / H) * 100
    # fax: sol kenarın üst kenara dik eksenden yatay sapması
    angle_ab = math.atan2(AB[1], AB[0])
    ca, sa = math.cos(-angle_ab), math.sin(-angle_ab)
    AD_rot_x = AD[0]*ca - AD[1]*sa
    AD_rot_y = AD[0]*sa + AD[1]*ca
    fax = AD_rot_x / AD_rot_y if abs(AD_rot_y) > 1e-6 else 0.0
    return {'pos_x': A[0], 'pos_y': A[1],
            'frz': frz, 'fscx': fscx, 'fscy': fscy, 'fax': fax}

def run_calculation(coords_str, ratio=None, size_str=None, alignment=7):
    """CLI ve GUI'nin ortak çağırdığı ana hesaplama fonksiyonu."""
    nums = parse_floats(coords_str)
    if len(nums) < 8:
        raise ValueError("8 koordinat gerekli (4 köşe x,y)")
    A=(nums[0],nums[1]); B=(nums[2],nums[3])
    C=(nums[4],nums[5]); D=(nums[6],nums[7])

    if size_str:
        wh = parse_floats(size_str); W,H = wh[0],wh[1]
        size_mode = "kullanıcı tanımlı"
    elif ratio:
        avg_h = (dist(A,D)+dist(B,C))/2; H=avg_h; W=H*ratio
        size_mode = f"oran {ratio}"
    else:
        W,H = estimate_real_size(A,B,C,D)
        size_mode = "otomatik tahmin"

    M    = homography_from_corners(A, B, C, D, W, H)
    tags = matrix_to_aegisub(M, A, B, C, D, W, H)

    an = alignment
    if an==5: tags['pos_x']=(A[0]+B[0]+C[0]+D[0])/4; tags['pos_y']=(A[1]+B[1]+C[1]+D[1])/4
    elif an==1: tags['pos_x'],tags['pos_y']=D
    elif an==3: tags['pos_x'],tags['pos_y']=C
    elif an==9: tags['pos_x'],tags['pos_y']=B

    t = tags
    tag_str = (f"{{\\an{an}"
               f"\\pos({t['pos_x']:.1f},{t['pos_y']:.1f})"
               f"\\frz{t['frz']:.2f}"
               f"\\fscx{t['fscx']:.1f}"
               f"\\fscy{t['fscy']:.1f}"
               f"\\fax{t['fax']:.4f}"
               f"}}Metin buraya")
    return {'A':A,'B':B,'C':C,'D':D,'W':W,'H':H,'size_mode':size_mode,
            'matrix':M,'tags':tags,'an':an,'tag_str':tag_str,
            'top':dist(A,B),'bottom':dist(D,C),'left':dist(A,D),'right':dist(B,C)}

#----------CLI----------

def cli_main(args):
    try:
        r = run_calculation(args.coord, ratio=args.target_ratio,
                            size_str=args.size, alignment=args.alignment)
    except ValueError as e:
        print(f"Hata: {e}"); sys.exit(1)

    print("\n── Köşe Noktaları ───────────────────────────────")
    for k,v in [("A (sol üst)",r['A']),("B (sağ üst)",r['B']),
                ("C (sağ alt)",r['C']),("D (sol alt)",r['D'])]:
        print(f"  {k} : {v}")
    print("\n── Kenar Uzunlukları (ekranda görünen) ──────────")
    for k,v in [("Üst",r['top']),("Alt",r['bottom']),("Sol",r['left']),("Sağ",r['right'])]:
        print(f"  {k:<5}: {v:.1f}px")
    print(f"\n── Gerçek Boyut ({r['size_mode']}) ──────────────")
    print(f"  W={r['W']:.1f}, H={r['H']:.1f}, oran={r['W']/r['H']:.3f}")
    print("\n── Homografi Matrisi ─────────────────────────────")
    for row in r['matrix']:
        print(f"  [{row[0]:10.5f}  {row[1]:10.5f}  {row[2]:10.5f}]")
    t = r['tags']
    print("\n── Aegisub Etiketleri ────────────────────────────")
    print(f"  \\an{r['an']}")
    print(f"  \\pos({t['pos_x']:.1f}, {t['pos_y']:.1f})")
    print(f"  \\frz{t['frz']:.2f}")
    print(f"  \\fscx{t['fscx']:.1f}")
    print(f"  \\fscy{t['fscy']:.1f}")
    print(f"  \\fax{t['fax']:.4f}")
    print("\n── Yapıştırmaya Hazır ────────────────────────────")
    print(f"  {r['tag_str']}")
    print("\n── İnce Ayar Rehberi ─────────────────────────────")
    print("  \\fscx  → yazı çok geniş/dar görünüyorsa ±5 adımla ayarla")
    print("  \\fscy  → yazı çok uzun/kısa görünüyorsa ±5 adımla ayarla")
    print("  \\fax   → sol/sağ kenar eğimi tutmuyorsa ±0.01 adımla ayarla")
    print("  \\frz   → üst kenar açısı tutmuyorsa ±0.5 adımla ayarla\n")

#----------GUI----------

DARK_BG="1a1a2e"; PANEL_BG="#16213e"; ACCENT="#e94560"; ACCENT2="#0f3460"
TEXT_LIGHT="#eaeaea"; TEXT_DIM="#8892a4"; INPUT_BG="#0d1b2a"; SUCCESS_BG="#0d2137"
DARK_BG="#1a1a2e"
FNT_TITLE=("Consolas",18,"bold"); FNT_LABEL=("Consolas",10)
FNT_INPUT=("Consolas",11); FNT_RESULT=("Consolas",11)
FNT_TAG=("Consolas",12,"bold"); FNT_SMALL=("Consolas",9)

def gui_main():
    import tkinter as tk
    from tkinter import messagebox

    def _clear(entry, hint):
        if entry.get()==hint: entry.delete(0,"end"); entry.config(fg=TEXT_LIGHT)
    def _restore(entry, hint):
        if entry.get()=="": entry.insert(0,hint); entry.config(fg=TEXT_DIM)

    class CornerEntry(tk.Frame):
        def __init__(self, parent, label, **kw):
            super().__init__(parent, bg=PANEL_BG, **kw)
            tk.Label(self, text=label, font=FNT_LABEL, fg=TEXT_DIM,
                     bg=PANEL_BG, width=12, anchor="w").pack(side="left", padx=(0,10))
            self.x_var=tk.StringVar(); self.y_var=tk.StringVar()
            for var, hint in [(self.x_var,"X"),(self.y_var,"Y")]:
                e = tk.Entry(self, textvariable=var, font=FNT_INPUT,
                             bg=INPUT_BG, fg=TEXT_LIGHT, insertbackground=ACCENT,
                             relief="flat", width=7, highlightthickness=1,
                             highlightbackground=ACCENT2, highlightcolor=ACCENT)
                e.pack(side="left", padx=(0,6))
                e.insert(0, hint)
                e.bind("<FocusIn>",  lambda ev,e=e,h=hint: _clear(e,h))
                e.bind("<FocusOut>", lambda ev,e=e,h=hint: _restore(e,h))
        def get(self):
            try: return (float(self.x_var.get()), float(self.y_var.get()))
            except ValueError: return None

    root = tk.Tk()
    root.title("Aegisub Perspektif Hesaplayıcı")
    root.configure(bg=DARK_BG)
    root.resizable(False, False)

    # İkon — pencere sol üstü + görev çubuğu
    import os, sys
    def _icon_path():
        # PyInstaller exe içinde _MEIPASS'a açar, normal çalışmada script dizini
        base = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(base, 'icon.ico')
    _ico = _icon_path()
    if os.path.exists(_ico):
        root.iconbitmap(_ico)

    # Başlık
    hdr = tk.Frame(root, bg=DARK_BG)
    hdr.pack(fill="x", padx=20, pady=(18,8))
    tk.Label(hdr, text="AEGISUB", font=FNT_TITLE, fg=ACCENT, bg=DARK_BG).pack(side="left")
    tk.Label(hdr, text=" Perspektif Hesaplayıcı", font=FNT_TITLE, fg=TEXT_LIGHT, bg=DARK_BG).pack(side="left")

    # Giriş paneli
    pnl = tk.Frame(root, bg=PANEL_BG, padx=18, pady=14)
    pnl.pack(fill="both", padx=20, pady=(0,4))

    def lbl(t, dim=False):
        tk.Label(pnl, text=t, font=FNT_LABEL, fg=TEXT_DIM if dim else TEXT_LIGHT, bg=PANEL_BG).pack(anchor="w")
    def sep():
        tk.Frame(pnl, height=1, bg=ACCENT2).pack(fill="x", pady=8)

    lbl("Köşe Koordinatları  (saat yönü: sol üst → sağ üst → sağ alt → sol alt)")
    tk.Frame(pnl, height=6, bg=PANEL_BG).pack()

    corners = {}
    for key, label in [("A","▸ Sol Üst"),("B","▸ Sağ Üst"),("C","▸ Sağ Alt"),("D","▸ Sol Alt")]:
        ce = CornerEntry(pnl, label)
        ce.pack(anchor="w", pady=2)
        corners[key] = ce

    sep()
    lbl("Gerçek Boyut  ('Otomatik' seçilirse program kendisi hesaplar)")
    tk.Frame(pnl, height=6, bg=PANEL_BG).pack()

    mode_var = tk.StringVar(value="auto")
    mode_row = tk.Frame(pnl, bg=PANEL_BG)
    mode_row.pack(fill="x")
    for text, val in [("Otomatik","auto"),("En:Boy Oranı","ratio"),("Piksel Boyutu","size")]:
        tk.Radiobutton(mode_row, text=text, variable=mode_var, value=val,
                       bg=PANEL_BG, fg=TEXT_LIGHT, selectcolor=ACCENT2,
                       activebackground=PANEL_BG, font=FNT_LABEL,
                       command=lambda: _on_mode()).pack(side="left", padx=(0,14))
    tk.Frame(pnl, height=6, bg=PANEL_BG).pack()

    ratio_frame = tk.Frame(pnl, bg=PANEL_BG); ratio_frame.pack(anchor="w")
    tk.Label(ratio_frame, text="En:Boy Oranı  ", font=FNT_LABEL, fg=TEXT_DIM, bg=PANEL_BG).pack(side="left")
    ratio_var = tk.StringVar(value="2.0")
    ratio_entry = tk.Entry(ratio_frame, textvariable=ratio_var, font=FNT_INPUT,
                           bg=INPUT_BG, fg=TEXT_LIGHT, insertbackground=ACCENT,
                           relief="flat", width=8, highlightthickness=1,
                           highlightbackground=ACCENT2, highlightcolor=ACCENT)
    ratio_entry.pack(side="left")
    tk.Label(ratio_frame, text="  (genişlik/yükseklik)",
             font=FNT_SMALL, fg=TEXT_DIM, bg=PANEL_BG).pack(side="left")

    size_frame = tk.Frame(pnl, bg=PANEL_BG); size_frame.pack(anchor="w")
    tk.Label(size_frame, text="Genişlik ", font=FNT_LABEL, fg=TEXT_DIM, bg=PANEL_BG).pack(side="left")
    size_w_var=tk.StringVar(value="1200"); size_h_var=tk.StringVar(value="400")
    for var in [size_w_var, size_h_var]:
        tk.Entry(size_frame, textvariable=var, font=FNT_INPUT, bg=INPUT_BG,
                 fg=TEXT_LIGHT, insertbackground=ACCENT, relief="flat", width=7,
                 highlightthickness=1, highlightbackground=ACCENT2,
                 highlightcolor=ACCENT).pack(side="left", padx=(0,4))
    tk.Label(size_frame, text="  Yükseklik", font=FNT_LABEL, fg=TEXT_DIM, bg=PANEL_BG).pack(side="left")

    def _on_mode():
        m = mode_var.get()
        for w in ratio_frame.winfo_children():
            try: w.config(state="normal" if m=="ratio" else "disabled")
            except: pass
        for w in size_frame.winfo_children():
            try: w.config(state="normal" if m=="size" else "disabled")
            except: pass
    _on_mode()
    sep()

    tk.Button(pnl, text="  HESAPLA  ", font=("Consolas",12,"bold"),
              bg=ACCENT, fg="white", relief="flat", activebackground="#c73652",
              activeforeground="white", cursor="hand2", padx=14, pady=6,
              command=lambda: _calculate()).pack(anchor="w")

    # Sonuç paneli
    res_outer = tk.Frame(root, bg=DARK_BG)
    res_outer.pack(fill="both", padx=20, pady=(4,16))
    res = tk.Frame(res_outer, bg=SUCCESS_BG, padx=18, pady=14)
    res.pack(fill="both")
    tk.Label(res, text="Sonuç", font=FNT_LABEL, fg=TEXT_DIM, bg=SUCCESS_BG).pack(anchor="w")
    tk.Frame(res, height=6, bg=SUCCESS_BG).pack()

    tag_vars = {}
    grid = tk.Frame(res, bg=SUCCESS_BG); grid.pack(anchor="w", fill="x")
    for i,(tag,desc) in enumerate([("\\pos","Konum"),("\\frz","Z rotasyonu"),
                                    ("\\fscx","Yatay ölçek"),("\\fscy","Dikey ölçek"),("\\fax","Yamuk")]):
        tk.Label(grid, text=tag, font=("Consolas",11,"bold"), fg=ACCENT,
                 bg=SUCCESS_BG, width=8, anchor="w").grid(row=i, column=0, sticky="w", pady=2)
        tk.Label(grid, text=desc, font=FNT_SMALL, fg=TEXT_DIM,
                 bg=SUCCESS_BG, width=14, anchor="w").grid(row=i, column=1, sticky="w")
        v=tk.StringVar(value="—"); tag_vars[tag]=v
        tk.Label(grid, textvariable=v, font=FNT_RESULT, fg=TEXT_LIGHT,
                 bg=SUCCESS_BG, anchor="w").grid(row=i, column=2, sticky="w", padx=(8,0))

    tk.Frame(res, height=10, bg=SUCCESS_BG).pack()
    tk.Label(res, text="Yapıştırmaya hazır:", font=FNT_LABEL, fg=TEXT_DIM, bg=SUCCESS_BG).pack(anchor="w")
    tk.Frame(res, height=4, bg=SUCCESS_BG).pack()

    tag_row = tk.Frame(res, bg=SUCCESS_BG); tag_row.pack(fill="x")
    full_tag_var = tk.StringVar(value="")
    tk.Entry(tag_row, textvariable=full_tag_var, font=FNT_TAG, bg=INPUT_BG, fg=ACCENT,
             insertbackground=ACCENT, relief="flat", highlightthickness=1,
             highlightbackground=ACCENT2, highlightcolor=ACCENT,
             state="readonly", readonlybackground=INPUT_BG, width=52
             ).pack(side="left", fill="x", expand=True)
    tk.Button(tag_row, text="Kopyala", font=FNT_LABEL, bg=ACCENT2, fg=TEXT_LIGHT,
              relief="flat", cursor="hand2", activebackground=ACCENT,
              activeforeground="white", padx=10,
              command=lambda: _copy()).pack(side="left", padx=(8,0))

    size_info_var = tk.StringVar(value="")
    tk.Label(res, textvariable=size_info_var, font=FNT_SMALL,
             fg=TEXT_DIM, bg=SUCCESS_BG).pack(anchor="w", pady=(6,0))

    def _calculate():
        pts = {}
        for k, ce in corners.items():
            pt = ce.get()
            if pt is None:
                messagebox.showerror("Hata", f"{k} köşesi için geçerli koordinat girin.")
                return
            pts[k] = pt
        coords_str = ",".join(f"{pts[k][0]},{pts[k][1]}" for k in ["A","B","C","D"])
        mode = mode_var.get()
        ratio = size_str = None
        if mode=="ratio":
            try: ratio=float(ratio_var.get())
            except ValueError: messagebox.showerror("Hata","Geçerli oran girin (örn: 2.5)"); return
        elif mode=="size":
            try: float(size_w_var.get()); float(size_h_var.get()); size_str=f"{size_w_var.get()}x{size_h_var.get()}"
            except ValueError: messagebox.showerror("Hata","Geçerli piksel değerleri girin."); return
        try:
            r = run_calculation(coords_str, ratio=ratio, size_str=size_str)
        except Exception as ex:
            messagebox.showerror("Hesaplama Hatası", str(ex)); return
        t = r['tags']
        tag_vars["\\pos"].set(f"({t['pos_x']:.1f}, {t['pos_y']:.1f})")
        tag_vars["\\frz"].set(f"{t['frz']:.2f}°")
        tag_vars["\\fscx"].set(f"{t['fscx']:.1f}%")
        tag_vars["\\fscy"].set(f"{t['fscy']:.1f}%")
        tag_vars["\\fax"].set(f"{t['fax']:.4f}")
        full_tag_var.set(r['tag_str'])
        size_info_var.set(f"Gerçek boyut: {r['W']:.0f}×{r['H']:.0f}px  "
                          f"(oran {r['W']/r['H']:.2f}:1)  —  {r['size_mode']}")

    def _copy():
        text = full_tag_var.get()
        if text:
            root.clipboard_clear(); root.clipboard_append(text)
            messagebox.showinfo("Kopyalandı","Etiket panoya kopyalandı.")

    root.update_idletasks()
    sw,sh=root.winfo_screenwidth(),root.winfo_screenheight()
    w,h=root.winfo_width(),root.winfo_height()
    root.geometry(f"+{(sw-w)//2}+{(sh-h)//2}")
    root.mainloop()

#----------GİRİŞ NOKTASI----------

def main():
    parser = argparse.ArgumentParser(
        description="Dörtgen köşelerinden Aegisub perspektif etiketleri üretir.")
    parser.add_argument("coord", nargs="?", default=None,
        metavar="<Koordinatlar>",
        help='8 koordinat: "x1,y1,x2,y2,x3,y3,x4,y4"')
    parser.add_argument("--gui", action="store_true", help="Grafik arayüzü aç")
    parser.add_argument("--ratio", type=float, default=None,
        metavar="R", dest="target_ratio", help="Gerçek en:boy oranı (örn: 2.5)")
    parser.add_argument("--size", type=str, default=None,
        metavar="WxH", help="Gerçek piksel boyutu (örn: 800x300)")
    parser.add_argument("--alignment", type=int, default=7,
        choices=[1,2,3,4,5,6,7,8,9], help="Aegisub hizalama (\\an). Varsayılan: 7")
    args = parser.parse_args()

    if args.gui or args.coord is None:
        gui_main()
    else:
        cli_main(args)

if __name__ == "__main__":
    main()