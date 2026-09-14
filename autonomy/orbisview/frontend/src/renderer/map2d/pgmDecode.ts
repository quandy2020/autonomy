/**
 * Decode Netpbm PGM (P2 ASCII / P5 binary) to opaque RGBA raster.
 * Avoids DOM ImageData so vitest/node can run without canvas.
 */
export interface PgmRaster {
  data: Uint8ClampedArray;
  width: number;
  height: number;
}

export function decodePgm(buffer: ArrayBuffer): PgmRaster {
  const bytes = new Uint8Array(buffer);
  let i = 0;
  const isSpace = (c: number) => c === 0x20 || c === 0x09 || c === 0x0a || c === 0x0d;

  const skipWsAndComments = () => {
    while (i < bytes.length) {
      if (bytes[i] === 0x23) {
        while (i < bytes.length && bytes[i] !== 0x0a) i++;
        continue;
      }
      if (isSpace(bytes[i])) {
        i++;
        continue;
      }
      break;
    }
  };

  const readToken = (): string => {
    skipWsAndComments();
    const start = i;
    while (i < bytes.length && !isSpace(bytes[i]) && bytes[i] !== 0x23) i++;
    let s = '';
    for (let k = start; k < i; k++) s += String.fromCharCode(bytes[k]);
    return s;
  };

  const magic = readToken();
  if (magic !== 'P2' && magic !== 'P5') {
    throw new Error(`Unsupported PGM magic: ${magic}`);
  }
  const width = Number(readToken());
  const height = Number(readToken());
  const maxVal = Number(readToken());
  if (!(width > 0) || !(height > 0) || !(maxVal > 0)) {
    throw new Error('Invalid PGM header');
  }

  skipWsAndComments();
  const out = new Uint8ClampedArray(width * height * 4);
  const scale = maxVal === 255 ? 1 : 255 / maxVal;

  if (magic === 'P5') {
    const needed = width * height;
    if (i + needed > bytes.length) throw new Error('Truncated P5 PGM');
    for (let p = 0; p < needed; p++) {
      const g = Math.round(bytes[i + p] * scale);
      const o = p * 4;
      out[o] = g;
      out[o + 1] = g;
      out[o + 2] = g;
      out[o + 3] = 255;
    }
  } else {
    let p = 0;
    while (p < width * height && i < bytes.length) {
      const tok = readToken();
      if (!tok) break;
      const g = Math.round(Number(tok) * scale);
      const o = p * 4;
      out[o] = g;
      out[o + 1] = g;
      out[o + 2] = g;
      out[o + 3] = 255;
      p++;
    }
    if (p !== width * height) throw new Error('Truncated P2 PGM');
  }

  return { data: out, width, height };
}
