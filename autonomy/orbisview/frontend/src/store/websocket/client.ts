import type { ServerMessage } from './types';

export type MessageHandler = (msg: ServerMessage) => void;
export type ConnState = 'offline' | 'connecting' | 'online' | 'reconnecting';
export type ConnHandler = (state: ConnState) => void;

export class OrbisWsClient {
  private ws: WebSocket | null = null;
  private handlers = new Set<MessageHandler>();
  private connHandlers = new Set<ConnHandler>();
  private url = 'ws://127.0.0.1:8766/ws';
  private intentionalClose = false;
  private retryMs = 500;
  private retryTimer: number | null = null;
  private state: ConnState = 'offline';

  onMessage(handler: MessageHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  onConnection(handler: ConnHandler): () => void {
    this.connHandlers.add(handler);
    handler(this.state);
    return () => this.connHandlers.delete(handler);
  }

  get connected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }

  get connectionState(): ConnState {
    return this.state;
  }

  private setState(state: ConnState): void {
    this.state = state;
    this.connHandlers.forEach((h) => h(state));
  }

  connect(url = this.url): void {
    this.url = url;
    this.intentionalClose = false;
    this.clearRetry();
    this.openSocket(false);
  }

  private openSocket(isRetry: boolean): void {
    if (this.ws) {
      this.ws.onopen = null;
      this.ws.onclose = null;
      this.ws.onerror = null;
      this.ws.onmessage = null;
      try {
        this.ws.close();
      } catch {
        /* ignore */
      }
      this.ws = null;
    }
    this.setState(isRetry ? 'reconnecting' : 'connecting');
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => {
      this.retryMs = 500;
      this.setState('online');
    };
    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(String(ev.data)) as ServerMessage;
        this.handlers.forEach((h) => h(msg));
      } catch {
        // ignore malformed
      }
    };
    ws.onclose = () => {
      this.ws = null;
      if (this.intentionalClose) {
        this.setState('offline');
        return;
      }
      this.setState('reconnecting');
      this.scheduleRetry();
    };
    ws.onerror = () => {
      // onclose will follow
    };
  }

  private scheduleRetry(): void {
    this.clearRetry();
    const delay = this.retryMs;
    this.retryMs = Math.min(8000, Math.floor(this.retryMs * 1.8));
    this.retryTimer = window.setTimeout(() => {
      if (!this.intentionalClose) this.openSocket(true);
    }, delay);
  }

  private clearRetry(): void {
    if (this.retryTimer != null) {
      window.clearTimeout(this.retryTimer);
      this.retryTimer = null;
    }
  }

  close(): void {
    this.intentionalClose = true;
    this.clearRetry();
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.setState('offline');
  }

  send(obj: Record<string, unknown>): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify(obj));
  }

  cmdVel(vx: number, wz: number): void {
    this.send({ op: 'cmd_vel', vx, wz });
  }

  listChannels(): void {
    this.send({ op: 'list_channels' });
  }

  subscribe(channel: string, maxHz = 20): void {
    this.send({ op: 'subscribe', channel, max_hz: maxHz });
  }

  unsubscribe(channel: string): void {
    this.send({ op: 'unsubscribe', channel });
  }

  status(): void {
    this.send({ op: 'status' });
  }
}

export const wsClient = new OrbisWsClient();
