import { get, writable, type Readable, type Writable, readonly, readable } from "svelte/store";
import { MessagePack } from "./MessagePack";

/**
 * Network tables type codes.
 */
export const NTTypeCodes = {
    boolean: 0,
    double: 1,
    int: 2,
    float: 3,
    string: 4,
    json: 4,
    raw: 5,
    rpc: 5,
    msgpack: 5,
    protobuf: 5,
    "boolean[]": 16,
    "double[]": 17,
    "int[]": 18,
    "float[]": 19,
    "string[]": 20,
} as const;

/**
 * Types supported by Network Tables.
 */
export type NTType = boolean | number | string | Uint8Array | boolean[] | number[] | string[];
/**
 * A string representing a topic's type.
 */
export type NTTypeString = keyof typeof NTTypeCodes;

/**
 * A parsed text data frame.
 */
export interface NTTextFrame {
    method: `publish` | `unpublish` | `setproperties` | `subscribe` | `unsubscribe` | `announce` | `unannounce` | `properties`;
    params: Record<string, any>;
}

/**
 * An NT server topic.
 */
export interface NTTopic {
    readonly name: string;
    readonly id: number;
    readonly type: NTTypeString;
    readonly pubuid?: number;
    readonly properties: Record<string, Readonly<any>>;
}

/**
 * NT Subscriber settings.
 */
export interface NTSubscriberSettings {
    /**
     * If true, the server should send all value changes over the wire. If false, only the most recent value is sent (same as NT 3.0 behavior).
     * @default true
     */
    readonly all: boolean;
    /**
     * How frequently the server should send changes. The server may send more frequently than this (e.g. use a combined minimum period for all values) or apply a restricted range to this value.
     * Specified in milliseconds.
     * @default 100
     */
    readonly periodic: number;
    /**
     * If topic history should be saved.
     * @default false
     */
    readonly saveHistory: boolean;
}

/**
 * An NT subscription that can be subscribed to a single topic.
 */
class NTSubscriber {
    private readonly _topicName: string;
    private readonly _settings: NTSubscriberSettings;

    private _history: Map<number, NTType | null> = new Map();
    private _listeners: Set<(value: NTType | null) => void> = new Set();
    private _subuid: number | null = null;
    private _topicId: number | null = null;
    private _value: NTType | null = null;
    private _valueTimestamp: number | null = null;

    /**
     * Creates the subscriber.
     * @param topicName The name of the topic.
     * @param settings Subscriber settings.
     */
    public constructor(topicName: string, settings: NTSubscriberSettings) {
        this._topicName = topicName;
        this._settings = settings;
    }

    /**
     * Gets the subscribe message JSON and sets the subscriber's internal state to be subscribed.
     * @param subuid The subscription UID to use.
     */
    public subscribe(subuid: number): NTTextFrame {
        this._subuid = subuid;
        return {
            method: `subscribe`,
            params: {
                subuid,
                topics: [this._topicName],
                options: {
                    all: this._settings.all,
                    periodic: this._settings.periodic / 1000,
                },
            },
        };
    }

    /**
     * Gets the unsubscribe message JSON and sets the subscriber's internal state to be unsubscribed.
     * Additionally, this sets the subscriber's value to `null`, updates the listeners, and clears the subscriber's history.
     */
    public unsubscribe(): NTTextFrame {
        const subuid = this._subuid;
        this._subuid = null;
        this._topicId = null;
        if (this._value !== null) this.updateValue(null);
        this._history.clear();
        return {
            method: `unsubscribe`,
            params: { subuid },
        };
    }

    /**
     * This should be called when the subscription's topic is announced.
     * @param topicId The topic ID sent by the server.
     */
    public onAnnounce(topicId: number): void {
        this._topicId = topicId;
    }

    /**
     * This should be called when the subscription's topic is unannounced.
     * Resets the saved subscription UID and topic ID, sets the subscriber's value to `null`, and updates listeners.
     * Topic history is still saved.
     */
    public onUnannounce(): void {
        this._subuid = null;
        this._topicId = null;
        if (this._value !== null) this.updateValue(null);
    }

    /**
     * This should be called when the client disconnects.
     */
    public onDisconnect(): void {
        this._subuid = null;
        this._topicId = null;
        if (this._value !== null) this.updateValue(null);
        this._history.clear();
    }

    /**
     * Gets the subscriber's subscription UID sent by the client.
     * This returns `null` if the subscriber hasn't been assigned a UID (via {@link NTSubscriber.subscribe()}), or if it was unsubscribed (via {@link NTSubscriber.unsubscribe()}).
     */
    public getSubuid(): number | null {
        return this._subuid;
    }

    /**
     * Gets the subscriber's topic ID sent by the server.
     * This returns `null` if the topic hasn't received an `announce` message, or if the topic was unannounced.
     */
    public getTopicId(): number | null {
        return this._topicId;
    }

    /**
     * Updates the subscriber's value and updates its listeners with the new value.
     * Also saves the value to the subscriptions's history if enabled.
     * @param value The new value.
     * @param timestamp The value's timestamp in milliseconds.
     */
    public updateValue(value: NTType | null, timestamp: number | null = null): void {
        if (timestamp === null || timestamp > (this._valueTimestamp ?? 0)) {
            this._value = value;
            this._valueTimestamp = timestamp;
            this._listeners.forEach((listener) => listener(value));
        }

        if (this._settings.saveHistory && timestamp !== null) {
            this._history.set(timestamp, value);
        }
    }

    /**
     * Adds a listener to the subscriber and invokes it with the current value.
     * @param listener The listener to add.
     */
    public addListener(listener: (value: NTType | null) => void): void {
        this._listeners.add(listener);
        listener(this._value);
    }

    /**
     * Removes a listener.
     * @param listener The listener to remove.
     * @returns If the subscriber contains any other listeners.
     */
    public removeListener(listener: (value: NTType | null) => void): boolean {
        this._listeners.delete(listener);
        return this._listeners.size > 0;
    }

    /**
     * Gets the subscriber's history.
     */
    public getHistory(): Map<number, NTType | null> {
        return this._history;
    }

    /**
     * Trims the subscriber's history back to a specified timestamp.
     * @param until Keep history as far back as this timestamp in milliseconds.
     */
    public trimHistory(until: number) {
        if (!this._settings.saveHistory) return;
        for (const [timestamp] of this._history) {
            if (timestamp < until) this._history.delete(timestamp);
        }
    }
}

/**
 * NT Publisher settings.
 */
export interface NTPublisherSettings {
    /**
     * If true, the last set value will be periodically saved to persistent storage on the server and be restored during server startup. Topics with this property set to true will not be deleted by the server when the last publisher stops publishing.
     * @default false
     */
    readonly persistent: boolean;
    /**
     * Topics with this property set to true will not be deleted by the server when the last publisher stops publishing.
     * @default false
     */
    readonly retained: boolean;
    /**
     * If false, the server and clients will not store the value of the topic. This means that only value updates will be available for the topic.
     * @default true
     */
    readonly cached: boolean;
}

/**
 * An NT publisher.
 */
class NTPublisher {
    private readonly _topicName: string;
    private readonly _type: NTTypeString;
    private readonly _properties: Record<string, any>;
    private readonly _settings: NTPublisherSettings;

    private _listeners: Set<(value: NTType) => void> = new Set();
    private _receivedAck: boolean = false;
    private _pubuid: number | null = null;
    private _value: NTType;

    /**
     * Creates the publisher.
     * @param topicName The name of the topic.
     * @param type The topic's type.
     * @param initialValue The initial value of the publisher.
     * @param settings Publisher settings.
     * @param properties Custom topic properties.
     */
    public constructor(
        topicName: string,
        type: NTTypeString,
        initialValue: NTType,
        settings: NTPublisherSettings,
        properties?: Record<string, any>,
    ) {
        this._topicName = topicName;
        this._type = type;
        this._value = initialValue;
        this._properties = properties ?? {};
        this._settings = settings;
    }

    /**
     * Gets the publish message JSON and sets the publisher's internal state to be published.
     * @param pubuid The publish UID to use.
     */
    public publish(pubuid: number): NTTextFrame {
        this._pubuid = pubuid;
        return {
            method: `publish`,
            params: {
                pubuid,
                name: this._topicName,
                type: this._type,
                properties: {
                    ...this._properties,
                    persistent: this._settings.persistent,
                    retained: this._settings.retained,
                    cached: this._settings.cached,
                },
            },
        };
    }

    /**
     * Gets the unpublish message JSON and sets the publisher's internal state to be unpublished.
     * This does not reset the saved value of the publisher.
     */
    public unpublish(): NTTextFrame {
        const pubuid = this._pubuid;
        this._pubuid = null;
        return {
            method: `unpublish`,
            params: { pubuid },
        };
    }

    /**
     * This should be called when the published topic is acknowledged by the server.
     */
    public onAck(): void {
        this._receivedAck = true;
    }

    /**
     * This should be called when the client disconnects.
     */
    public onDisconnect(): void {
        this._pubuid = null;
        this._receivedAck = false;
    }

    /**
     * Gets the publisher's publish UID sent by the client.
     * This returns `null` if the publisher hasn't been assigned a UID (via {@link NTPublisher.publish()}), or if it was unpublished (via {@link NTPublisher.unpublish()}).
     */
    public getPubuid(): number | null {
        return this._pubuid;
    }

    /**
     * Returns `true` if the published topic was acknowledged by the server.
     */
    public getAck(): boolean {
        return this._receivedAck;
    }

    /**
     * Updates the publisher's client-side value and updates its listeners with the new value.
     * @param value The new value.
     */
    public updateValue(value: NTType): void {
        this._value = value;
        this._listeners.forEach((listener) => listener(value));
    }

    /**
     * Adds a listener to the publisher and invokes it with the current value.
     * @param listener The listener to add.
     */
    public addListener(listener: (value: NTType) => void): void {
        this._listeners.add(listener);
        listener(this._value);
    }

    /**
     * Removes a listener.
     * @param listener The listener to remove.
     * @returns If the publisher contains any other listeners.
     */
    public removeListener(listener: (value: NTType) => void): boolean {
        this._listeners.delete(listener);
        return this._listeners.size > 0;
    }

    /**
     * Returns a binary frame representing the publisher's current value.
     * Returns `null` if a publisher UID has not been assigned to the publisher, or if it has a value of `null`.
     * @param serverTimeUs The current server time (in microseconds).
     */
    public getBinaryFrame(serverTimeUs: number): any[] | null {
        if (typeof this._pubuid !== `number` || this._value === null) return null;
        else return [this._pubuid, serverTimeUs, NTTypeCodes[this._type], this._value];
    }
}

/**
 * NT Client state.
 */
export enum NTSvelteClientState {
    IDLE,
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
}

/**
 * NT Client settings.
 */
export interface NTSvelteClientSettings {
    /**
     * The name for the client. Used to identify the connection in robot logs.
     * @default `GRRDashboard`
     */
    readonly appName: string;
    /**
     * The TTL for a value in a topic's history in milliseconds. Has no effect when a subscriber's `saveHistory` is `false`.
     * Set to `0` to disable.
     * @default 0
     */
    readonly historyTTL: number;
    /**
     * Default subscriber settings.
     */
    readonly subscribers: NTSubscriberSettings;
    /**
     * Default publisher settings.
     */
    readonly publishers: NTPublisherSettings;
}

/**
 * A Network Tables 4.1 client with bindings for Svelte.
 */
export class NTSvelteClient {
    private static readonly _HISTORY_SWEEP_PERIOD = 100;
    private static readonly _OPEN_TIMEOUT = 5000;
    private static readonly _PORT = 5810;
    private static readonly _RECONNECT_DELAY = 1500;
    private static readonly _RTT_PERIOD = 500;
    private static readonly _SERVER_AVAILABLE_TIMEOUT = 500;
    private static readonly _WS_PROTOCOL = `v4.1.networktables.first.wpi.edu`;
    private static readonly _WS_RTT_PROTOCOL = `rtt.networktables.first.wpi.edu`;

    private readonly _settings: NTSvelteClientSettings;

    private _bitrate: Writable<number> = writable(0);
    private _latency: Writable<number> = writable(0);
    private _openTimeout: number | null = null;
    private _serverTime: Writable<number> = writable();
    private _state: Writable<NTSvelteClientState> = writable(NTSvelteClientState.IDLE);
    private _uidNonce: number = 0;
    private _uri: Writable<string>;

    private _ws: WebSocket | null = null;
    private _wsRtt: WebSocket | null = null;
    private _wsListeners = {
        open: [() => this._onOpen(false), () => this._onOpen(true)],
        close: [(event: CloseEvent) => this._onClose(event, false), (event: CloseEvent) => this._onClose(event, true)],
        error: [() => this._onError(false), () => this._onError(true)],
        message: [(event: MessageEvent) => this._onMessage(event, false), (event: MessageEvent) => this._onMessage(event, true)],
    };

    private _aliveAck = false;
    private _serverOffsetUs: number = 0;
    private _usage: number = 0;

    private _topicsOnlySubuid: number | null = null;
    private _publishers: Map<string, NTPublisher> = new Map();
    private _subscribers: Map<string, NTSubscriber> = new Map();
    private _serverTopics: Writable<Map<string, NTTopic>> = writable(new Map());

    /**
     * Creates the NT Svelte Client.
     * Use {@link NTSvelteClient.connect()} to start the connection.
     * @param uri The URI to connect to.
     * @param settings Client settings.
     */
    public constructor(uri: string, settings?: Partial<NTSvelteClientSettings>) {
        this._uri = writable(uri);
        this._settings = {
            appName: settings?.appName ?? `GRRDashboard`,
            historyTTL: settings?.historyTTL ?? 0,
            subscribers: {
                all: settings?.subscribers?.all ?? true,
                periodic: settings?.subscribers?.periodic ?? 100,
                saveHistory: settings?.subscribers?.saveHistory ?? false,
            },
            publishers: {
                persistent: settings?.publishers?.persistent ?? false,
                retained: settings?.publishers?.retained ?? false,
                cached: settings?.publishers?.cached ?? true,
            },
        };

        setInterval(() => {
            if (get(this._state) === NTSvelteClientState.CONNECTED) {
                if (!this._aliveAck) return this._restart(`Timed out`);
                this._aliveAck = false;
                this._sendTimestamp();
                this._bitrate.set(this._usage / (NTSvelteClient._RTT_PERIOD / 1000));
                this._usage = 0;
            }
        }, NTSvelteClient._RTT_PERIOD);

        if (this._settings.historyTTL > 0) {
            setInterval(() => {
                this._subscribers.forEach((subscriber) => {
                    subscriber.trimHistory(Date.now() - this._settings.historyTTL);
                });
            }, NTSvelteClient._HISTORY_SWEEP_PERIOD);
        }
    }

    /**
     * Sets a new URI and restarts the client.
     * @param newURI The new URI.
     */
    public setURI(newURI: string) {
        this._uri.set(newURI);
        this._restart(`URI Changed`);
    }

    /**
     * Connect to the NT server.
     */
    public connect(): void {
        if (get(this._state) === NTSvelteClientState.IDLE) {
            this._state.set(NTSvelteClientState.DISCONNECTED);
            this._connect();
        }
    }

    /**
     * Gets the URI in use by the client, as a readable.
     */
    public uriReadable(): Readable<string> {
        return readonly(this._uri);
    }

    /**
     * Gets the state of the client, as a readable.
     */
    public stateReadable(): Readable<NTSvelteClientState> {
        return readonly(this._state);
    }

    /**
     * Gets the bitrate between the client and server (bits/s), as a readable.
     */
    public bitrateReadable(): Readable<number> {
        return readonly(this._bitrate);
    }

    /**
     * Gets the latency between the client and server (ms), as a readable.
     */
    public latencyReadable(): Readable<number> {
        return readonly(this._latency);
    }

    /**
     * Gets the server time in milliseconds, as a readable.
     */
    public serverTimeReadable(): Readable<number> {
        return readonly(this._serverTime);
    }

    /**
     * Gets a map of topics announced by the server, as a readable.
     */
    public topicsReadable(): Readable<Map<string, NTTopic>> {
        return readonly(this._serverTopics);
    }

    /**
     * Subscribes to all topics with the `topicsonly` flag set to `true`.
     * Useful for observing all topics available on the server without receiving value changes.
     */
    public subAllNoValues(): void {
        if (this._topicsOnlySubuid === null || this._topicsOnlySubuid < 0) {
            this._topicsOnlySubuid = this._genUID();
            const sent = this._sendJSON({
                method: `subscribe`,
                params: {
                    subuid: this._topicsOnlySubuid,
                    topics: [``],
                    options: {
                        periodic: this._settings.subscribers.periodic / 1000,
                        topicsonly: true,
                        prefix: true,
                    },
                },
            });
            if (!sent) this._topicsOnlySubuid = -1;
        }
    }

    /**
     * Removes a subscription created from {@link NTSvelteClient.subAllNoValues()}.
     */
    public unsubAllNoValues(): void {
        if (this._topicsOnlySubuid !== null && this._topicsOnlySubuid >= 0) {
            const subuid = this._topicsOnlySubuid;
            this._topicsOnlySubuid = null;
            this._sendJSON({ method: `unsubscribe`, params: { subuid } });
        }
    }

    /**
     * Subscribes to a NT topic.
     * @param key The key of the topic to subscribe to.
     * @param defaultValue The default value, returned when the topic is unavailable.
     * @param settings Subscription settings. Ignored if a subscription with the same key is currently active.
     * @returns The value of the topic as a readable.
     */
    public subscribe<T extends NTType>(key: string, defaultValue: T, settings?: Partial<NTSubscriberSettings>): Readable<T> {
        return readable(defaultValue, (set) => {
            const listener = (value: any) => set(value ?? defaultValue);

            let sub = this._subscribers.get(key);
            if (sub) {
                sub.addListener(listener);
            } else {
                sub = new NTSubscriber(key, {
                    all: settings?.all ?? this._settings.subscribers.all,
                    periodic: settings?.periodic ?? this._settings.subscribers.periodic,
                    saveHistory: settings?.saveHistory ?? this._settings.subscribers.saveHistory,
                });
                sub.addListener(listener);
                sub.updateValue(null);
                const existingTopic = get(this._serverTopics).get(key);
                if (existingTopic) sub.onAnnounce(existingTopic.id);
                this._subscribers.set(key, sub);
                this._sendJSON(sub.subscribe(this._genUID()));
            }

            return () => {
                if (!sub) return;
                const others = sub.removeListener(listener);
                if (!others) {
                    this._sendJSON(sub.unsubscribe());
                    this._subscribers.delete(key);
                }
            };
        });
    }

    /**
     * Publishes a NT topic.
     * To modify the published topic's properties,
     * @param key The key of the topic to publish.
     * @param type The topic's type.
     * @param initialValue The topic's initial value. If the topic is already published, its value is updated to this.
     * @param settings Publish settings. Ignored if a publisher with the same key is currently active.
     * @param properties Custom topic properties. Ignored if a publisher with the same key is currently active.
     * @returns The publisher as a writable. Its value is updated if other writables assigned to the same publisher update their value.
     */
    public publish<T extends NTType>(
        key: string,
        type: NTTypeString,
        initialValue: T,
        settings?: Partial<NTPublisherSettings>,
        properties?: Record<string, any>,
    ): Writable<T> {
        const store = writable(initialValue, (set) => {
            const listener = (value: any) => set(value);

            let pub = this._publishers.get(key);
            if (pub) {
                pub.addListener(listener);
                pub.updateValue(initialValue);
            } else {
                pub = new NTPublisher(
                    key,
                    type,
                    initialValue,
                    {
                        persistent: settings?.persistent ?? this._settings.publishers.persistent,
                        retained: settings?.retained ?? this._settings.publishers.retained,
                        cached: settings?.cached ?? this._settings.publishers.cached,
                    },
                    properties,
                );
                pub.addListener(listener);
                pub.updateValue(initialValue);
                if (typeof get(this._serverTopics).get(key)?.pubuid === `number`) pub.onAck();
                this._publishers.set(key, pub);
                this._sendJSON(pub.publish(this._genUID()));
            }

            if (pub.getAck()) {
                const frame = pub.getBinaryFrame(this._getServerTimeUs());
                if (Array.isArray(frame)) this._sendMessagePack(frame, false);
            }

            return () => {
                if (!pub) return;
                const others = pub.removeListener(listener);
                if (!others) {
                    this._sendJSON(pub.unpublish());
                    this._publishers.delete(key);
                }
            };
        });

        return {
            subscribe: store.subscribe,
            update: store.update,
            set: (value: T) => {
                const pub = this._publishers.get(key);
                pub?.updateValue(value);
                if (pub?.getAck()) {
                    const frame = pub.getBinaryFrame(this._getServerTimeUs());
                    if (Array.isArray(frame)) this._sendMessagePack(frame, false);
                }
            },
        };
    }

    /**
     * Gets the URL used for checking if the server is alive.
     */
    private get _aliveUrl(): string {
        return encodeURI(`http://${get(this._uri)}:${NTSvelteClient._PORT}`);
    }

    /**
     * Gets the server's WebSocket URL.
     */
    private get _wsUrl(): string {
        return encodeURI(`ws://${get(this._uri)}:${NTSvelteClient._PORT}/nt/${this._settings.appName}`);
    }

    /**
     * Gets the server's current time (in microseconds).
     */
    private _getServerTimeUs(): number {
        return Date.now() * 1000 + this._serverOffsetUs;
    }

    /**
     * Sends the client's timestamp via the RTT connection.
     */
    private _sendTimestamp() {
        this._sendMessagePack([-1, 0, NTTypeCodes.int, Date.now() * 1000], true);
    }

    /**
     * Sends a message pack packet.
     * @param data The data to send.
     * @param rtt If the packet should be sent to the RTT connection.
     * @returns If the data was sent.
     */
    private _sendMessagePack(data: any[], rtt: boolean): boolean {
        const ws = rtt ? this._wsRtt : this._ws;
        if (ws?.readyState === WebSocket.OPEN) {
            const msg = MessagePack.serialize(data);
            ws.send(msg);
            this._usage += msg.byteLength * 8;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sends a JSON message.
     * Only used for the primary WS connection, not the RTT connection.
     * @param method The method value to send.
     * @param params JSON parameters.
     * @returns If the data was sent.
     */
    private _sendJSON(data: NTTextFrame): boolean {
        if (this._ws?.readyState === WebSocket.OPEN) {
            const msg = JSON.stringify([data]);
            this._ws.send(msg);
            this._usage += this._getStringSize(msg);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Connect to the server.
     * @param reconnecting If the client is reconnecting.
     */
    private async _connect(reconnecting: boolean = false): Promise<void> {
        if (get(this._state) !== NTSvelteClientState.DISCONNECTED) return;
        if (this._openTimeout !== null) clearTimeout(this._openTimeout);
        this._state.set(NTSvelteClientState.CONNECTING);

        if (reconnecting) {
            console.log(`[NTSvelte] Waiting to reconnect...`);
            await new Promise((resolve) => setTimeout(resolve, NTSvelteClient._RECONNECT_DELAY));
        }

        console.log(`[NTSvelte] Connecting...`);

        let aliveResult: Response | null = null;
        let length = `0.00`;

        try {
            const start = Date.now();
            aliveResult = await fetch(this._aliveUrl, { signal: AbortSignal.timeout(NTSvelteClient._SERVER_AVAILABLE_TIMEOUT) });
            length = ((Date.now() - start) / 1000).toFixed(2);
        } catch (_) {}

        if (!aliveResult?.ok) {
            this._state.set(NTSvelteClientState.DISCONNECTED);
            console.warn(`[NTSvelte] Server not responding after ${length}s while attempting to connect`);
            return this._connect(true);
        }

        console.log(`[NTSvelte] Server responded after ${length}s`);

        this._openTimeout = setTimeout(() => this._restart(`Timed out while connecting`), NTSvelteClient._OPEN_TIMEOUT);

        this._ws = new WebSocket(this._wsUrl, [NTSvelteClient._WS_PROTOCOL]);
        this._ws.binaryType = `arraybuffer`;

        this._wsRtt = new WebSocket(this._wsUrl, [NTSvelteClient._WS_RTT_PROTOCOL]);
        this._wsRtt.binaryType = `arraybuffer`;

        for (const [type, listener] of Object.entries(this._wsListeners)) {
            this._ws.addEventListener(type, listener[0] as any);
            this._wsRtt.addEventListener(type, listener[1] as any);
        }
    }

    /**
     * Closes and restart the connection.
     * @param reason The reason for restarting.
     */
    private _restart(reason: string): void {
        if (this._openTimeout !== null) clearTimeout(this._openTimeout);
        this._openTimeout = null;

        for (const [type, listener] of Object.entries(this._wsListeners)) {
            this._ws?.removeEventListener(type, listener[0] as any);
            this._wsRtt?.removeEventListener(type, listener[1] as any);
        }

        this._ws?.close();
        this._ws = null;
        this._wsRtt?.close();
        this._wsRtt = null;

        this._bitrate.set(0);
        this._latency.set(0);
        this._serverTime.set(0);

        this._aliveAck = false;
        this._serverOffsetUs = 0;
        this._usage = 0;

        if (this._topicsOnlySubuid !== null) this._topicsOnlySubuid = -1;
        this._subscribers.forEach((subscriber) => subscriber.onDisconnect());
        this._publishers.forEach((publisher) => publisher.onDisconnect());
        this._serverTopics.set(new Map());

        this._state.set(NTSvelteClientState.DISCONNECTED);
        console.warn(`[NTSvelte] Restarting: ${reason}`);
        this._connect(true);
    }

    private _onOpen(rtt: boolean): void {
        console.log(`[NTSvelte] ${rtt ? `RTT ` : ``}WebSocket Connected`);

        if (this._ws?.readyState === WebSocket.OPEN && this._wsRtt?.readyState === WebSocket.OPEN) {
            if (this._openTimeout !== null) clearTimeout(this._openTimeout);
            this._openTimeout = null;
            this._aliveAck = true;
            this._state.set(NTSvelteClientState.CONNECTED);
            console.log(`[NTSvelte] Connection complete`);

            this._sendTimestamp();

            if (this._topicsOnlySubuid !== null) this.subAllNoValues();

            this._subscribers.forEach((subscriber) => {
                if (subscriber.getSubuid() === null) this._sendJSON(subscriber.subscribe(this._genUID()));
            });

            this._publishers.forEach((publisher) => {
                if (publisher.getPubuid() === null) this._sendJSON(publisher.publish(this._genUID()));
            });
        }
    }

    private _onClose(event: CloseEvent, rtt: boolean): void {
        this._restart(
            `${rtt ? `RTT ` : ``}WebSocket Closed with code ${event.code}: ${event.reason?.length ? event.reason : `Unknown reason`}`,
        );
    }

    private _onError(rtt: boolean): void {
        this._restart(`${rtt ? `RTT ` : ``}Websocket encountered an error`);
    }

    private _onMessage(event: MessageEvent, rtt: boolean): void {
        const now = Date.now() * 1000;

        if (typeof event.data === `string`) {
            if (rtt) return console.warn(`[NTSvelte] Received unexpected text frame from RTT connection`);

            this._usage += this._getStringSize(event.data);
            let data: NTTextFrame[] = JSON.parse(event.data);
            if (!Array.isArray(data)) return console.warn(`[NTSvelte] Received malformed text frame, data is not an array`);

            data.forEach((frame, i) => {
                if (typeof frame !== `object`)
                    return console.warn(`[NTSvelte] Received malformed text frame, frame at index ${i} is not an object`);

                const method = frame.method;
                const params = frame.params;
                if (typeof method !== `string`)
                    return console.warn(
                        `[NTSvelte] Received malformed text frame, frame at index ${i} has invalid method of type ${typeof method}`,
                    );
                if (typeof params !== `object`)
                    return console.warn(
                        `[NTSvelte] Received malformed text frame, frame at index ${i} has invalid params of type ${typeof params}`,
                    );

                switch (method) {
                    case `announce`:
                        this._serverTopics.set(
                            new Map(
                                get(this._serverTopics).set(
                                    params.name,
                                    this._deepFreeze({
                                        id: params.id,
                                        name: params.name,
                                        type: params.type,
                                        pubuid: params.pubuid,
                                        properties: params.properties ?? {},
                                    }),
                                ),
                            ),
                        );

                        this._subscribers.get(params.name)?.onAnnounce(params.id);
                        if (typeof params.pubuid === `number`) {
                            const pub = this._publishers.get(params.name);
                            if (pub?.getPubuid() === params.pubuid && !pub.getAck()) {
                                pub.onAck();
                                const frame = pub.getBinaryFrame(this._getServerTimeUs());
                                if (Array.isArray(frame)) this._sendMessagePack(frame, false);
                            }
                        }
                        break;
                    case `unannounce`:
                        const topics = get(this._serverTopics);
                        topics.delete(params.name);
                        this._serverTopics.set(new Map(topics));

                        this._subscribers.get(params.name)?.onUnannounce();
                        break;
                    case `properties`:
                        const topic = get(this._serverTopics).get(params.name);
                        if (!topic) return console.warn(`[NTSvelte] Ignoring properties update, topic was not announced`);

                        const properties = structuredClone(topic.properties);
                        for (const [k, v] of Object.entries((params.update ?? {}) as NTTopic[`properties`])) {
                            if (v !== null) properties[k] = v;
                            else delete properties[k];
                        }

                        this._serverTopics.set(
                            new Map(get(this._serverTopics).set(params.name, this._deepFreeze({ ...topic, properties }))),
                        );
                        break;
                    default:
                        console.warn(`[NTSvelte] Received unknown method "${method}" in text frame at index ${i}`);
                        break;
                }
            });
        } else if (event.data instanceof ArrayBuffer) {
            this._usage += event.data.byteLength * 8;

            MessagePack.deserialize(event.data, { multiple: true }).forEach((unpacked: any[]) => {
                const topicId: number = unpacked[0];
                const timestamp: number = unpacked[1];
                const type: (typeof NTTypeCodes)[NTTypeString] = unpacked[2];
                const value: NTType = unpacked[3];

                if (typeof topicId !== `number`)
                    return console.warn(`[NTSvelte] Received malformed binary frame, topic ID is not a number`);
                if (typeof timestamp !== `number`)
                    return console.warn(`[NTSvelte] Received malformed binary frame, timestamp is not a number`);
                if (typeof type !== `number`) return console.warn(`[NTSvelte] Received malformed binary frame, type index is not a number`);

                if (topicId === -1) {
                    this._aliveAck = true;
                    const usLatency = (now - (value as number)) / 2;
                    const serverTimeUs = timestamp + usLatency;
                    this._latency.set(usLatency / 1000);
                    this._serverTime.set(serverTimeUs / 1000);
                    this._serverOffsetUs = serverTimeUs - now;
                    return;
                }

                if (rtt) return console.warn(`[NTSvelte] Received unexpected binary frame with topic ID ${topicId} on RTT connection`);

                if (topicId >= 0) {
                    let found = false;
                    this._subscribers.forEach((subscriber) => {
                        if (subscriber.getTopicId() === topicId) {
                            subscriber.updateValue(value, timestamp / 1000);
                            found = true;
                        }
                    });
                    if (!found) console.warn(`[NTSvelte] Received binary frame with unknown topic ID ${topicId}`);
                } else {
                    return console.warn(`[NTSvelte] Received malformed binary frame, topic ID "${topicId}" out of range`);
                }
            });
        } else {
            console.warn(`[NTSvelte] Received unknown message type`);
        }
    }

    /**
     * Generates a new UID.
     */
    private _genUID(): number {
        return this._uidNonce++;
    }

    /**
     * Gets the number of bits in a string.
     * @param str The string.
     */
    private _getStringSize(str: string): number {
        return (encodeURI(str).split(/%..|./).length - 1) * 8;
    }

    /**
     * Deep freezes an object in place.
     * Supports circular references.
     * @param obj The object to freeze.
     * @returns The object.
     */
    private _deepFreeze<T extends Record<string, any>>(obj: T): T {
        Object.freeze(obj);
        Object.values(obj)
            .filter((o) => typeof o === `object` && !Object.isFrozen(o))
            .forEach((o) => this._deepFreeze(o));
        return obj;
    }
}
