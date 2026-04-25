interface PendulumWasm {
  _engine_create(
    N: number,
    paramsPtr: number,
    thetaPtr: number,
    thetaDotPtr: number
  ): number;
  _engine_destroy(handle: number): void;
  _engine_advance(handle: number, dt: number): void;
  _engine_time(handle: number): number;
  _engine_energy(handle: number): number;
  _engine_energy_drift(handle: number): number;
  _engine_initial_energy(handle: number): number;
  _engine_get_n(handle: number): number;
  _engine_get_state(
    handle: number,
    thetaPtr: number,
    thetaDotPtr: number
  ): void;
  _engine_get_positions(
    handle: number,
    posPtr: number,
    lengthsPtr: number
  ): void;
  _malloc(size: number): number;
  _free(ptr: number): void;
  setValue(ptr: number, value: number, type: string): void;
  getValue(ptr: number, type: string): number;
}

export interface LinkConfig {
  L: number;
  m_rod: number;
  m_bob: number;
  r_bob: number;
}

export interface PendulumState {
  theta: number[];
  theta_dot: number[];
  t: number;
  energy: number;
  energy_drift: number;
  positions: { x: number; y: number }[];
}

export class PendulumEngine {
  private wasm: PendulumWasm | null = null;
  private handle: number = 0;
  private _N: number;
  private _links: LinkConfig[];
  private _g: number;
  private _theta0: number[];
  private _thetaDot0: number[];

  // Pre-allocated WASM buffers for getState (avoid malloc/free per frame)
  private _thetaBuf = 0;
  private _thetaDotBuf = 0;
  private _posBuf = 0;
  private _lengthsBuf = 0;

  constructor(
    links: LinkConfig[],
    theta0: number[],
    thetaDot0: number[],
    g: number = 9.80665
  ) {
    this._N = links.length;
    this._links = links;
    this._g = g;
    this._theta0 = theta0;
    this._thetaDot0 = thetaDot0;
  }

  private static wasmModule: PendulumWasm | null = null;
  private static wasmLoading: Promise<PendulumWasm> | null = null;

  private static loadWasm(): Promise<PendulumWasm> {
    if (PendulumEngine.wasmModule)
      return Promise.resolve(PendulumEngine.wasmModule);
    if (PendulumEngine.wasmLoading) return PendulumEngine.wasmLoading;

    PendulumEngine.wasmLoading = new Promise<PendulumWasm>((resolve) => {
      const script = document.createElement("script");
      script.src = "/wasm/pendulum.js";
      script.onload = async () => {
        const mod = await (window as any).PendulumModule();
        PendulumEngine.wasmModule = mod;
        resolve(mod);
      };
      document.head.appendChild(script);
    });
    return PendulumEngine.wasmLoading;
  }

  async init(): Promise<void> {
    this.wasm = await PendulumEngine.loadWasm();
    this.createEngine();
  }

  private setF64(ptr: number, index: number, value: number): void {
    this.wasm!.setValue(ptr + index * 8, value, "double");
  }

  private getF64(ptr: number, index: number): number {
    return this.wasm!.getValue(ptr + index * 8, "double");
  }

  private createEngine(): void {
    if (!this.wasm) throw new Error("WASM not loaded");

    const N = this._N;
    const paramsPtr = this.wasm._malloc((N * 5 + 1) * 8);
    const thetaPtr = this.wasm._malloc(N * 8);
    const thetaDotPtr = this.wasm._malloc(N * 8);

    for (let i = 0; i < N; i++) {
      this.setF64(paramsPtr, i * 5 + 0, this._links[i].L);
      this.setF64(paramsPtr, i * 5 + 1, this._links[i].m_rod);
      this.setF64(paramsPtr, i * 5 + 2, this._links[i].m_bob);
      this.setF64(paramsPtr, i * 5 + 3, this._links[i].r_bob);
      this.setF64(paramsPtr, i * 5 + 4, 0);
    }
    this.setF64(paramsPtr, N * 5, this._g);

    for (let i = 0; i < N; i++) {
      this.setF64(thetaPtr, i, this._theta0[i]);
      this.setF64(thetaDotPtr, i, this._thetaDot0[i]);
    }

    this.handle = this.wasm._engine_create(N, paramsPtr, thetaPtr, thetaDotPtr);

    this.wasm._free(paramsPtr);
    this.wasm._free(thetaPtr);
    this.wasm._free(thetaDotPtr);

    // Pre-allocate reusable buffers for getState
    this._thetaBuf = this.wasm._malloc(N * 8);
    this._thetaDotBuf = this.wasm._malloc(N * 8);
    this._posBuf = this.wasm._malloc(N * 2 * 8);
    this._lengthsBuf = this.wasm._malloc(N * 8);
    for (let i = 0; i < N; i++)
      this.setF64(this._lengthsBuf, i, this._links[i].L);
  }

  advance(dt: number): void {
    if (!this.wasm || !this.handle) return;
    this.wasm._engine_advance(this.handle, dt);
  }

  getState(): PendulumState {
    if (!this.wasm || !this.handle)
      return {
        theta: [],
        theta_dot: [],
        t: 0,
        energy: 0,
        energy_drift: 0,
        positions: [],
      };

    const N = this._N;
    this.wasm._engine_get_state(this.handle, this._thetaBuf, this._thetaDotBuf);
    this.wasm._engine_get_positions(this.handle, this._posBuf, this._lengthsBuf);

    const theta: number[] = [];
    const theta_dot: number[] = [];
    const positions: { x: number; y: number }[] = [];

    for (let i = 0; i < N; i++) {
      theta.push(this.getF64(this._thetaBuf, i));
      theta_dot.push(this.getF64(this._thetaDotBuf, i));
      positions.push({
        x: this.getF64(this._posBuf, 2 * i),
        y: this.getF64(this._posBuf, 2 * i + 1),
      });
    }

    return {
      theta,
      theta_dot,
      t: this.wasm._engine_time(this.handle),
      energy: this.wasm._engine_energy(this.handle),
      energy_drift: this.wasm._engine_energy_drift(this.handle),
      positions,
    };
  }

  destroy(): void {
    if (this.wasm) {
      if (this._thetaBuf) this.wasm._free(this._thetaBuf);
      if (this._thetaDotBuf) this.wasm._free(this._thetaDotBuf);
      if (this._posBuf) this.wasm._free(this._posBuf);
      if (this._lengthsBuf) this.wasm._free(this._lengthsBuf);
      this._thetaBuf = this._thetaDotBuf = this._posBuf = this._lengthsBuf = 0;
      if (this.handle) {
        this.wasm._engine_destroy(this.handle);
        this.handle = 0;
      }
    }
  }

  get N(): number {
    return this._N;
  }
  get links(): LinkConfig[] {
    return this._links;
  }
}
