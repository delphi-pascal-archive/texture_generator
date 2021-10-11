unit Utexture;

interface

uses
  Windows, Messages, SysUtils, Classes, Graphics, Controls, Forms, Dialogs;

type
  PLongA = ^TLongA;
  TLongA = array [0..0] of LongWord;
  PByteA = ^TByteA;
  TByteA = array [Word] of Byte;
  PRGBQuadA = ^TRGBQuadA;
  TRGBQuadA = array [Word] of TRGBQuad;
  TLongArray = array of LongWord;

type
  TForm1 = class(TForm)
    procedure FormCreate(Sender: TObject);
    procedure FormPaint(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
  private
    bmg : TBitmap;
    NrFrame:   Integer;
    st2, startTime: double;
    Map,Map1: TLongArray;
    Pal: array[0..255] of TRGBQuad;

    procedure CreateMap;

    procedure InitNoise(seed: Integer);
    function  noise1D(const x: double): double;
    function noise2D(const x, y: double): double;
    function noise3D(const x, y, z: double): double;
    function turbulence1D(x: double; const n: integer): double;
    function turbulence2D(x, y: double; const n: integer): double;
    function turbulence3D(x, y, z: double; const n: integer): double;
    function turbulence2Di(x, y: Integer; const n: integer): Integer;
    function noise2Di(const x, y: Integer): Integer;
    function turbulence3Di(x, y, z: Integer; const n: integer): Integer;
    function noise3Di(const x, y, z: Integer): Integer;

    procedure CreatePalette;
    function GenerateMap(const SizeX, SizeY: Word;
                         ScaleX: double = 0;
                         ScaleY: double = 0;
                         Octaves: integer = -1;
                         OffsetX: double = 0;
                         OffSetY: double = 0): TLongArray; overload;
    procedure GenerateMap(Map: TLongArray;
                         const SizeX, SizeY: Word;
                         ScaleX: double = 0;
                         ScaleY: double = 0;
                         Octaves: integer = -1;
                         OffsetX: double = 0;
                         OffSetY: double = 0); overload;
    function turbulence2Dia(x, y: Integer; const n: integer): Integer;
    procedure ColorizeBaseMap(Map: TLongArray; Color: integer = 0);
    procedure PowMap(Map: TLongArray);
    procedure MultMap(Map1, Map2: TLongArray);

  public
    procedure WMEraseBkgnd(var Message: TWMEraseBkgnd); message WM_ERASEBKGND;
  end;

const
  MapSizeX = 400;
  MapSizeY = 400;

const
  B:  Integer = $100;
  BM: Integer = $FF;
  N:  Integer = $1000;
  Ni:  Integer = $400000;
  NM: Integer = $FFF;
  NP: Integer = 12;
  B2: Integer = $202; // B + B + 2;
var
  P:  array[0..$202] of Integer;
  G1: array[0..$202] of Double;
  G2: array[0..$202,0..1] of Double;
  G2i: array[0..$202,0..1] of Integer;
  G3: array[0..$202,0..2] of Double;
  G3i: array[0..$202,0..2] of Integer;

var
  Form1: TForm1;

implementation

{$R *.DFM}

uses
  math, mmsystem;

procedure TForm1.FormCreate(Sender: TObject);
begin
  Randomize;
  startTime := timeGetTime * 0.001;
  NrFrame := 0;

  bmg := Tbitmap.Create;
  bmg.Width := MapSizeX;
  bmg.Height := MapSizeY;
  bmg.PixelFormat := pf32bit;
  ClientWidth := MapSizeX;
  ClientHeight := MapSizeY;

  SetLength(Map, MapSizeX * MapSizeY);
  SetLength(Map1, MapSizeX * MapSizeY);

  CreatePalette;
  InitNoise(100);
  st2 := now;
end;


function TForm1.GenerateMap(const SizeX, SizeY: Word; ScaleX,
  ScaleY: double; Octaves: integer; OffsetX, OffSetY: double): TLongArray;
var
  map: TLongArray;
begin
  if (SizeX * SizeY) = 0 then
    Exit;

  SetLength(Map, Sizex * SizeY);

  GenerateMap(Map, SizeX, SizeY, ScaleX, ScaleY, Octaves, OffsetX, OffsetY);

  Result := map;
end;

procedure TForm1.GenerateMap(Map: TLongArray; const SizeX, SizeY: Word; ScaleX,
  ScaleY: double; Octaves: integer; OffsetX, OffSetY: double);
var
  x,y,i: integer;
  v: integer;
  Xi,Yi,DXi,DYi: integer;
begin
  if (SizeX * SizeY) = 0 then
    Exit;

  if High(Map) <> (Sizex * SizeY -1) then
    SetLength(Map, Sizex * SizeY);

  if octaves < 0 then
    Octaves := Round(random * 8);

  if ScaleX = 0 then
    ScaleX := sqrt(random + 0.5);

  if ScaleY = 0 then
    ScaleY := sqrt(random + 0.5);

  if OffsetX = 0 then
    OffsetX := random * 100;

  if OffsetY = 0 then
    OffsetY := random * 100;

  Xi := Round(OffsetX * 1024);
  Yi := Round(OffsetY * 1024);
  DXI := Round(1024 * ScaleX/SizeX);
  DYI := Round(1024 * ScaleY/SizeY);

  i := 0;
  for y := 0 to SizeY - 1 do begin
    for x := SizeX - 1 downto 0 do begin
      v := turbulence2Dia(XI,YI, Octaves) shr 2;

      map[i] := (v shl 16) + (v shl 8) + v;
      Inc(i);
      Inc(XI, DXI);
    end;
    XI := Round(OffsetX * 1024);
    Inc(YI, DYI);
  end;
end;

procedure TForm1.ColorizeBaseMap(Map: TLongArray; Color: integer);
var
  pal: array[0..255] of LongWord;
  r,g,b: Integer;
  i: integer;
begin
  if Color <> 0 then begin
    r := (Color and $FF0000) shr 16;
    g := (Color and $FF00) shr 8;
    b := Color and $FF;
  end else begin
    repeat
      r := Round(random * 255);
      g := Round(random * 255);
      b := Round(random * 255);
    until ((r + g + b) > 550){ and ((r + g + b) < 250)};
  end;

  for i:= 0 to 255 do begin
    Pal[i] := round(min((i*r)/127,255)) shl 16 +
              round(min((i*g)/127,255)) shl 8 +
              round(min((i*b)/127,255));
  end;

  for i := 0 to High(Map) do begin
    Map[i] := Pal[Map[i] and $FF];
  end;
end;

procedure TForm1.PowMap(Map: TLongArray);
var
  r,g,b,v,i: integer;
begin
  for i := 0 to High(Map) do begin
    v := Map[i];
    r := 255 - ((v and $FF0000) shr 16);
    g := 255 - ((v and $FF00) shr 8);
    b := 255 - (v and $FF);

    r := r * r div 256;
    g := g * g div 256;
    b := b * b div 256;
    map[i] := r shl 16 + g shl 8 + b;
  end;
end;

procedure TForm1.MultMap(Map1, Map2: TLongArray);
var
  v,w,i: integer;
  rv,gv,bv,rw,gw,bw,r,g,b: integer;
begin
  for i := 0 to High(Map) do begin
    v := Map1[i];
    w := Map2[i];
    rv := (v and $FF0000) shr 16;
    gv := (v and $FF00) shr 8;
    bv := v and $FF;
    rw := (w and $FF0000) shr 16;
    gw := (w and $FF00) shr 8;
    bw := w and $FF;
    r := min(255, (((rv * 3) div 2 + 127) * rw) div 256);
    g := min(255, (((gv * 3) div 2 + 127) * gw) div 256);
    b := min(255, (((bv * 3) div 2 + 127) * bw) div 256);
    map1[i] := r shl 16 + g shl 8 + b;
  end;
end;

procedure TForm1.CreateMap;
var
  x,y,i: longword;
  Row: PRGBQuadA;
begin
  GenerateMap(Map, MapSizeX, MapSizeY);
  ColorizeBaseMap(Map);
  PowMap(Map);
  GenerateMap(Map1, MapSizeX, MapSizeY);
  ColorizeBaseMap(Map1);
  PowMap(Map1);
  MultMap(Map, Map1);
  GenerateMap(Map1, MapSizeX, MapSizeY);
  ColorizeBaseMap(Map1);
  PowMap(Map1);
  MultMap(Map, Map1);

  i := 0;
  for y := 0 to MapSizeY - 1 do begin
    Row := bmg.Scanline[y];
    for x := 0 to MapSizeX - 1 do begin
      Row[x] := tRGBQUAD(map[i]);
      Inc(i);
    end;
  end;

(*
  setLength(Map1,MapSize * MapSize);
  setLength(Map2,MapSize * MapSize);
  setLength(Map3,MapSize * MapSize);
  fx1 := 0.3 + random * 0.67;
  fy1 := 0.3 + random * 0.67;

  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := round(255 * abs(turbulence2D(fx1 * x/MapSize, fy1 * y/MapSize , 2)));
      v := (v shl 16) + (v shl 8) + v;
      map1[y * MapSize + x] := v;
    end;
  end;

  fx2 := 0.55 + random * 0.35;
  fy2 := 0.55 + random * 0.35;
  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := round(255 * abs(turbulence2D(fx2 * x/MapSize + 100, fy2 * y/MapSize + 200 , 4)));
      v := (v shl 16) + (v shl 8) + v;
      map2[y * MapSize + x] := v;
    end;
  end;

  fx3 := 0.4 + random * 1.2;
  fy3 := 0.4 + random * 1.2;
  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := round(255 * abs(turbulence2D(fx2 * x/MapSize + 200, fy2 * y/MapSize + 300 , 8)));
      v := (v shl 16) + (v shl 8) + v;
      map3[y * MapSize + x] := v;
    end;
  end;

  //colorize
//        0xa0a0a0, 0x804060, 0xc08020, 0x90d030, 0x406070, 0xa98765, 0x346790, 0xaabbcc

  r := $c0;
  g := $80;
  b := $20;
  for i:= 0 to 255 do begin
    ra[i] := round(min((i*r)/127,255)) shl 16;
    ga[i] := round(min((i*g)/127,255)) shl 8;
    ba[i] := round(min((i*b)/127,255));
  end;

  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := map1[y * MapSize + x];
      map1[y * MapSize + x] := ra[(v and $FF0000) shr 16] + ga[(v and $FF00) shr 8] + ba[(v and $FF)];
    end;
  end;

  r := $aa;
  g := $bb;
  b := $cc;
  for i:= 0 to 255 do begin
    ra[i] := round(min((i*r)/127,255)) shl 16;
    ga[i] := round(min((i*g)/127,255)) shl 8;
    ba[i] := round(min((i*b)/127,255));
  end;

  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := map2[y * MapSize + x];
      map2[y * MapSize + x] := ra[(v and $FF0000) shr 16] + ga[(v and $FF00) shr 8] + ba[(v and $FF)];
    end;
  end;

  r := $90;
  g := $d0;
  b := $30;
  for i:= 0 to 255 do begin
    ra[i] := round(min((i*r)/127,255)) shl 16;
    ga[i] := round(min((i*g)/127,255)) shl 8;
    ba[i] := round(min((i*b)/127,255));
  end;

  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := map3[y * MapSize + x];
      map3[y * MapSize + x] := ra[(v and $FF0000) shr 16] + ga[(v and $FF00) shr 8] + ba[(v and $FF)];
    end;
  end;

  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := map3[y * MapSize + x];
      r := (v and $FF0000) shr 16;
      g := (v and $FF00) shr 8;
      b := v and $FF;
      r := (256 - r) * (256 - r) div 256;
      g := (256 - g) * (256 - g) div 256;
      b := (256 - b) * (256 - b) div 256;
      v := r shl 16 + g shr 8 + b;
      map3[y * MapSize + x] := v;
    end;
  end;
{
  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := map1[y * MapSize + x];
      w := map2[y * MapSize + x];
      rv := (v and $FF0000) shr 16;
      gv := (v and $FF00) shr 8;
      bv := v and $FF;
      rw := (w and $FF0000) shr 16;
      gw := (w and $FF00) shr 8;
      bw := w and $FF;
      r := min(255, (((rv * 3) div 2 + 127) * rw) div 256);
      g := min(255, (((gv * 3) div 2 + 127) * gw) div 256);
      b := min(255, (((bv * 3) div 2 + 127) * bw) div 256);
      map1[y * MapSize + x] := r shl 16 + g shl 8 + b;
    end;
  end;
{
  for y := 0 to MapSize -1 do begin
    for x := MapSize -1 downto 0 do begin
      v := map1[y * MapSize + x];
      w := map3[y * MapSize + x];
      rv := (v and $FF0000) shr 16;
      gv := (v and $FF00) shr 8;
      bv := v and $FF;
      rw := (w and $FF0000) shr 16;
      gw := (w and $FF00) shr 8;
      bw := w and $FF;
      r := min(255, (((rv * 3) div 2 + 127) * rw) div 256);
      g := min(255, (((gv * 3) div 2 + 127) * gw) div 256);
      b := min(255, (((bv * 3) div 2 + 127) * bw) div 256);
      map1[y * MapSize + x] := r shl 16 + g shl 8 + b;
    end;
  end;
}
*)

end;

procedure TForm1.FormPaint(Sender: TObject);
var
  s: double;
begin
  CreateMap;

  Canvas.Draw(0,0,bmg);

  Inc(NrFrame);
  s := (timeGetTime * 0.001) - StartTime;
  if s > 2 then begin
    Caption := Format('%.2f Frames per second', [NrFrame/s]);

    startTime := timeGetTime * 0.001;
    NrFrame := 0;
  end;

  invalidate;
end;

procedure TForm1.WMEraseBkgnd(var Message: TWMEraseBkgnd);
begin
  Message.Result := 1;
end;

procedure TForm1.FormDestroy(Sender: TObject);
begin
  bmg.free;
end;

function TForm1.noise1D(const x: double): double;
var
  bx0,bx1: integer;
  rx0,rx1,sx,t,u,v: double;
begin
  t := x + N;
  bx0 := Trunc(t) and BM;
  bx1 := (bx0 + 1) and BM;
  rx0 := t - trunc(t);
  rx1 := rx0 - 1;

  sx := rx0 * rx0 * (3 - 2 * rx0);
  u  := rx0 * G1[P[bx0]];
  v  := rx1 * G1[P[bx1]];
  result := u + sx * (v - u);
end;

function TForm1.noise2D(const x,y : double): double;
var
  bx0,bx1,by0,by1: integer;
  b00,b10,b01,b11: integer;
  rx0, rx1, ry0, ry1: double;
  sx, sy,t,a,b,u,v: double;
begin
  t := x + N;
  bx0 := Trunc(t) and BM;
  bx1 := (bx0 + 1) and BM;
  rx0 := t - trunc(t);
  rx1 := rx0 - 1;

  t := y + N;
  by0 := Trunc(t) and BM;
  by1 := (by0 + 1) and BM;
  ry0 := t - trunc(t);
  ry1 := ry0 - 1;

  b00 := P[P[bx0]+by0];
  b10 := P[P[bx1]+by0];
  b01 := P[P[bx0]+by1];
  b11 := P[P[bx1]+by1];

  sx := rx0 * rx0 * (3 - 2 * rx0);
  sy := ry0 * ry0 * (3 - 2 * ry0);

  u := rx0 * G2[b00][0] + ry0 * G2[b00][1];
  v := rx1 * G2[b10][0] + ry0 * G2[b10][1];
  a := u + sx * (v - u);

  u := rx0 * G2[b01][0] + ry1 * G2[b01][1];
  v := rx1 * G2[b11][0] + ry1 * G2[b11][1];
  b := u + sx * (v - u);

  result := a + sy * (b - a);
end;

function TForm1.noise2Di(const x,y: Integer): Integer;
var
  bx0,bx1,by0,by1: integer;
  b00,b10,b01,b11: integer;
  rx0, rx1, ry0, ry1: integer;
  sx, sy,t,a,b,u,v: integer;
  i,j: integer;
  r: integer;
begin
  t := x + $40000000;
  bx0 := (t shr 10) and $FF;
  bx1 := (bx0 + 1) and $FF;
  i := P[bx0];
  rx0 := t and 1023;
  rx1 := rx0 - 1024;

  t := y + $40000000;
  by0 := (t shr 10) and $FF;
  by1 := (by0 + 1) and $FF;
  j := P[bx1];
  ry0 := t and 1023;
  ry1 := ry0 - 1024;

  b00 := P[i + by0];
  b10 := P[j + by0];
  b01 := P[i + by1];
  b11 := P[j + by1];

  sx := (rx0 * rx0 * (3072 - 2 * rx0)) shr 20;
  sy := (ry0 * ry0 * (3072 - 2 * ry0)) shr 20;

  u := (rx0 * G2i[b00][0] + ry0 * G2i[b00][1]);
  v := (rx1 * G2i[b10][0] + ry0 * G2i[b10][1]);
  a := (u shl 10) + sx * (v - u);
asm
  SAR a,20
end;

  u := (rx0 * G2i[b01][0] + ry1 * G2i[b01][1]);
  v := (rx1 * G2i[b11][0] + ry1 * G2i[b11][1]);
  b := (u shl 10)  + sx * (v - u);
asm
  SAR b,20
end;

  r := a shl 10  + sy * (b - a);
asm
  SAR r,10
end;

  result := r;
end;


function TForm1.noise3Di(const x,y,z: Integer): Integer;
var
  bx0,bx1,by0,by1,bz0,bz1: integer;
  b00,b10,b01,b11: integer;
  rx0, rx1, ry0, ry1, rz0, rz1: integer;
  sx, sy, sz, t,a,b,c,d,u,v: integer;
  i,j: integer;
  r: integer;
begin
  t := x + $40000000;
  bx0 := (t shr 10) and $FF;
  bx1 := (bx0 + 1) and $FF;
  i := P[bx0];
  rx0 := t and 1023;
  rx1 := rx0 - 1024;

  t := y + $40000000;
  by0 := (t shr 10) and $FF;
  by1 := (by0 + 1) and $FF;
  j := P[bx1];
  ry0 := t and 1023;
  ry1 := ry0 - 1024;

  t := z + $40000000;
  bz0 := (t shr 10) and $FF;
  bz1 := (bz0 + 1) and $FF;
  rz0 := t and 1023;
  rz1 := rz0 - 1024;

  b00 := P[i + by0];
  b10 := P[j + by0];
  b01 := P[i + by1];
  b11 := P[j + by1];

  sx := (rx0 * rx0 * (3072 - 2 * rx0)) shr 20;
  sy := (ry0 * ry0 * (3072 - 2 * ry0)) shr 20;
  sz := (rz0 * rz0 * (3072 - 2 * rz0)) shr 20;

  u := rx0 * G3i[b00 + bz0][0] + ry0 * G3i[b00 + bz0][1] + rz0 * G3i[b00 + bz0][2];
  v := rx1 * G3i[b10 + bz0][0] + ry0 * G3i[b10 + bz0][1] + rz0 * G3i[b10 + bz0][2];
  a := (u shl 10) + sx * (v - u);
asm
  SAR a,20
end;

  u := rx0 * G3i[b01 + bz0][0] + ry1 * G3i[b01 + bz0][1] + rz0 * G3i[b01 + bz0][2];
  v := rx1 * G3i[b11 + bz0][0] + ry1 * G3i[b11 + bz0][1] + rz0 * G3i[b11 + bz0][2];
  b := (u shl 10)  + sx * (v - u);
asm
  SAR b,20
end;

  c := a shl 10 + sy * (b - a);

  u := rx0 * G3i[b00 + bz1][0] + ry0 * G3i[b00 + bz1][1] + rz1 * G3i[b00 + bz1][2];
  v := rx1 * G3i[b10 + bz1][0] + ry0 * G3i[b10 + bz1][1] + rz1 * G3i[b10 + bz1][2];
  a := u shl 10 + sx * (v - u);
asm
  SAR a,20
end;

  u := rx0 * G3i[b01 + bz1][0] + ry1 * G3i[b01 + bz1][1] + rz1 * G3i[b01 + bz1][2];
  v := rx1 * G3i[b11 + bz1][0] + ry1 * G3i[b11 + bz1][1] + rz1 * G3i[b11 + bz1][2];
  b := u shl 10 + sx * (v - u);
asm
  SAR b,20
end;

  d := a shl 10 + sy * (b - a);

  r := c shl 10  + sz * (d - c);
asm
  SAR r,20
end;

  result := r;
end;


function TForm1.noise3D(const x,y,z : double): double;
var
  bx0,bx1,by0,by1,bz0,bz1: integer;
  b00,b10,b01,b11: integer;
  rx0, rx1, ry0, ry1, rz0, rz1: double;
  sx,sy,sz, t,a,b,c,d,u,v: double;
begin
  t := x + N;
  bx0 := Trunc(t) and BM;
  bx1 := (bx0 + 1) and BM;
  rx0 := t - trunc(t);
  rx1 := rx0 - 1;

  t := y + N;
  by0 := Trunc(t) and BM;
  by1 := (by0 + 1) and BM;
  ry0 := t - trunc(t);
  ry1 := ry0 - 1;

  t := z + N;
  bz0 := Trunc(t) and BM;
  bz1 := (bz0 + 1) and BM;
  rz0 := t - trunc(t);
  rz1 := rz0 - 1;

  b00 := P[P[bx0]+by0];
  b10 := P[P[bx1]+by0];
  b01 := P[P[bx0]+by1];
  b11 := P[P[bx1]+by1];

  sx := rx0 * rx0 * (3 - 2 * rx0);
  sy := ry0 * ry0 * (3 - 2 * ry0);
  sz := rz0 * rz0 * (3 - 2 * rz0);

  u := rx0 * G3[b00 + bz0][0] + ry0 * G3[b00 + bz0][1] + rz0 * G3[b00 + bz0][2];
  v := rx1 * G3[b10 + bz0][0] + ry0 * G3[b10 + bz0][1] + rz0 * G3[b10 + bz0][2];
  a := u + sx * (v - u);

  u := rx0 * G3[b01 + bz0][0] + ry1 * G3[b01 + bz0][1] + rz0 * G3[b01 + bz0][2];
  v := rx1 * G3[b11 + bz0][0] + ry1 * G3[b11 + bz0][1] + rz0 * G3[b11 + bz0][2];
  b := u + sx * (v - u);

  c := a + sy * (b - a);

  u := rx0 * G3[b00 + bz1][0] + ry0 * G3[b00 + bz1][1] + rz1 * G3[b00 + bz1][2];
  v := rx1 * G3[b10 + bz1][0] + ry0 * G3[b10 + bz1][1] + rz1 * G3[b10 + bz1][2];
  a := u + sx * (v - u);

  u := rx0 * G3[b01 + bz1][0] + ry1 * G3[b01 + bz1][1] + rz1 * G3[b01 + bz1][2];
  v := rx1 * G3[b11 + bz1][0] + ry1 * G3[b11 + bz1][1] + rz1 * G3[b11 + bz1][2];
  b := u + sx * (v - u);

  d := a + sy * (b - a);

  result := c + sz * (d - c);
end;

procedure TForm1.initnoise(seed: Integer);
var
  I,J,T: integer;
  len: double;
begin
//  randseed := seed;
  randomize;

  for i := 0 to B - 1 do begin
    P[i] := i;

    G1[i] := (Trunc(Random * 2 * B) - B)/B;

    G2[i,0] := (Trunc(Random * 2 * B) - B)/B;
    G2[i,1] := (Trunc(Random * 2 * B) - B)/B;
    len := sqrt(G2[i,0] * G2[i,0] + G2[i,1] * G2[i,1]);
    if len > 1E-5 then begin
      G2[i,0] := G2[i,0] / len;
      G2[i,1] := G2[i,1] / len;
    end;

    G2i[i,0] := trunc(G2[i,0] * 1024);
    G2i[i,1] := trunc(G2[i,1] * 1024);

    G3[i,0] := (Trunc(Random * 2 * B) - B)/B;
    G3[i,1] := (Trunc(Random * 2 * B) - B)/B;
    G3[i,2] := (Trunc(Random * 2 * B) - B)/B;
    len := sqrt(G3[i,0] * G3[i,0] + G3[i,1] * G3[i,1] + G3[i,2] * G3[i,2]);
    if len > 1E-5 then begin
      G3[i,0] := G3[i,0] / len;
      G3[i,1] := G3[i,1] / len;
      G3[i,2] := G3[i,2] / len;
    end;

    G3i[i,0] := trunc(G3[i,0] * 1024);
    G3i[i,1] := trunc(G3[i,1] * 1024);
    G3i[i,2] := trunc(G3[i,2] * 1024);
  end;

  for i := 0 to B - 1 do begin
    j := Trunc(Random * B);
    T := P[i];
    P[i] := P[j];
    P[j] := T;
  end;

  for i := 0 to B + 1 do begin
    P[B + i] := P[i];

    G1[B + i] := G1[i];

    G2[B + i][0] := G2[i][0];
    G2[B + i][1] := G2[i][1];

    G2i[B + i][0] := G2i[i][0];
    G2i[B + i][1] := G2i[i][1];

    G3[B + i][0] := G3[i][0];
    G3[B + i][1] := G3[i][1];
    G3[B + i][2] := G3[i][2];

    G3i[B + i][0] := G3i[i][0];
    G3i[B + i][1] := G3i[i][1];
    G3i[B + i][2] := G3i[i][2];
  end;
end;

function TForm1.turbulence1D(x: double; const n: integer): double;
var
  freq: double;
  i: integer;
begin
  result := 0;
  freq := 1.0;

  for i := n - 1 downto 0 do begin
    result := result + Noise1D(x) * freq;
    x := x * 2;
    freq := freq * 0.5;
  end;
end;

function TForm1.turbulence2D(x,y: double; const n: integer): double;
var
  freq: double;
  i: integer;
begin
  result := 0;
  freq := 1.0;

  for i := n - 1 downto 0 do begin
    result := result + abs(Noise2D(x,y)) * freq;
    x := x * 2;
    y := y * 2;
    freq := freq * 0.5;
  end;
end;

function TForm1.turbulence2Di(x,y: Integer; const n: integer): Integer;
var
  r, i: integer;
  a: integer;
begin
  r := 0;
  a := 1;

  for i := n - 1 downto 0 do begin
    Inc(r, Noise2Di(x,y) div a);
    x := x shl 1;
    y := y shl 1;
    a := a shl 1;
  end;
  Result := abs(r);
end;

function TForm1.turbulence2Dia(x,y: Integer; const n: integer): Integer;
var
  r, i: integer;
  a: integer;
begin
  r := 0;
  a := 1;

  for i := n - 1 downto 0 do begin
    Inc(r, abs(Noise2Di(x,y)) div a);
    x := x shl 1;
    y := y shl 1;
    a := a shl 1;
  end;
  Result := r;
end;

function TForm1.turbulence3D(x,y,z: double; const n: integer): double;
var
  freq: double;
  i: integer;
begin
  result := 0;
  freq := 1.0;

  for i := n - 1 downto 0 do begin
    result := result + Noise3D(x,y,z) * freq;
    x := x * 2;
    y := y * 2;
    z := z * 2;
    freq := freq * 0.5;
  end;
  result := abs(Result)
end;

function TForm1.turbulence3Di(x,y,z: Integer; const n: integer): Integer;
var
  r, i: integer;
  a: integer;
begin
  r := 0;
  a := 1;

  for i := n - 1 downto 0 do begin
    Inc(r, Noise3Di(x,y,z) div a);
    x := x shl 1;
    y := y shl 1;
    z := z shl 1;
    a := a shl 1;
  end;
  Result := abs(r);
//  Result := r;
end;

procedure TForm1.CreatePalette;
var
  i: integer;
begin
  for i := 0 to 255 do begin
    Pal[i].rgbRed   := i;
    Pal[i].rgbGreen := i;
    Pal[i].rgbBlue  := round(sqrt(i/255) * 255);
  end;
end;

end.
