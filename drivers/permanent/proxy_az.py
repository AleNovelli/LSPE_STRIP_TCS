#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "proxy.py"
#
#Description:
#   This program acts as a modbus proxy between the programs launched by the workstation and the TrioController.
#   It allows multiple connections to the same controller on a first come first served basis
#
#**********************************************************************************************************

import asyncio  # serve a far funzionare più parti di codice contemporaneamente
import pathlib  # gestisce i path sia Windows che Linux
import logging.config  # sostituisce i print con messaggi più carini e li salva sulla console
from urllib.parse import urlparse  # separa l'url in diverse parti (ip, porta, etc.)

__version__ = "0.4.2"

DEFAULT_LOG_CONFIG = {
    "version": 1,
    "formatters": {
        "standard": {
            "format": "%(asctime)s %(levelname)8s %(name)s: %(message)s"
        }
    },
    "handlers": {
        "console": {
            "class": "logging.StreamHandler",
            "formatter": "standard",
        }
    },
    "root": {
        "handlers": ['console'],
        "level": "WARNING"
    }
}

log = None


class Connection:

    def __init__(self, name, reader, writer):
        self.name = name
        self.reader = reader
        self.writer = writer
        self.log = log.getChild(name)

    async def __aenter__(self):  # è lo start di asyncio
        return self

    async def __aexit__(self, exc_type, exc_value, tb):  # è lo stop di asyncio
        await self.close()

    @property
    def opened(self):
        # controlo se la connessione è aperta (devo avere un writer che non stia venendo chiuso e un reader not at end of file)
        return self.writer is not None and not self.writer.is_closing() and not self.reader.at_eof()

    async def close(self):  # chiudo la connessione
        if self.writer is not None:  # se ho un writer
            self.log.info("closing connection...")
            try:
                self.writer.close()  # provo a chiudere la connessione
                await self.writer.wait_closed()
            except Exception as error:
                self.log.info("failed to close: %r", error)
            else:
                self.log.info("connection closed")
            finally:
                self.reader = None
                self.writer = None

    async def _write(self, data):  # poco chiara la differenza tra _write() e write()
        # (forse _write() non chiude la connessione e write() si)
        self.log.debug("sending %r", data)
        self.writer.write(data)  # scrivo i dati sul buffer usando il writer
        await self.writer.drain()  # e aspetto che siano stati tutti inviati

    async def write(self, data):  # poca chiara la differenza con _write
        # (forse _write() non chiude la connessione e write() si)
        try:
            await self._write(data)  # provo a scrivere sulla connessione
        except Exception as error:
            self.log.error("writing error: %r", error)
            await self.close()  # e poi chiudo la connessione
            return False
        return True

    async def _read(self):  # stesso problema tra _read() e read() che ho con write e write()
        """Read ModBus TCP message"""
        # TODO: Handle Modbus RTU and ASCII #---> (modbus RTU è se si usa la seriale mentre ASCII è poco chiaro)
        header = await self.reader.readexactly(6)  # leggo l'header del messaggio
        size = int.from_bytes(header[4:], "big")  # salvo la dimensione del messaggio
        reply = header + await self.reader.readexactly(size)  # leggo il resto del messaggio
        self.log.debug("received %r", reply)
        return reply

    async def read(self):  # stesso problema tra _read() e read() che ho con write e write()
        try:
            return await self._read()  # leggo un messaggio lungo la connessione
        except asyncio.IncompleteReadError as error:
            if error.partial:
                self.log.error("reading error: %r", error)
            else:
                self.log.info("client closed connection")
            await self.close()
        except Exception as error:
            self.log.error("reading error: %r", error)
            await self.close()


class Client(Connection):  # the client class derives from the connection class
    # non è altro che una connessione qualunque con un fancy constructor
    def __init__(self, reader, writer):
        peer = writer.get_extra_info("peername")
        super().__init__(f"Client({peer[0]}:{peer[1]})", reader, writer)
        self.log.info("new client connection")


class ModBus(Connection):  # the modbus class derives from the connection class
    # credo che questo sia per il server e quindi sia un po' più complesso del client

    # inizializzazione di una modbus connection
    def __init__(self, host, port, modbus_host, modbus_port, timeout=None, connection_time=0.1):
        super().__init__(f"ModBus({modbus_host}:{modbus_port})", None, None)  # inizializzo una connessione
        # senza però specificare reader e writer

        self.host = host  # ip a cui si collega il client
        self.port = port  # porta a cui si collega il client
        self.modbus_host = modbus_host  # ip del modbus a cui va ridiretto il traffico
        self.modbus_port = modbus_port  # porta del modbus a cui va ridiretto il traffico
        self.timeout = timeout
        self.connection_time = connection_time
        self.lock = asyncio.Lock()  # An asyncio lock can be used to guarantee exclusive access to a shared resource.

    # aprire la connessione al modbus
    async def open(self):
        self.log.info("connecting to modbus...")
        self.reader, self.writer = \
            await asyncio.open_connection(self.modbus_host, self.modbus_port)
        self.log.info("connected!")
        # apro la connessione al modbus e salvo il reader e il writer

    # credo che questa funzione gestisca sia il read che il write ma non sono sicuro
    async def write_read(self, data, attempts=2):
        async with self.lock:
            for i in range(attempts):
                try:
                    if not self.opened:
                        await asyncio.wait_for(self.open(), self.timeout)
                        if self.connection_time > 0:
                            await asyncio.sleep(self.connection_time)
                    coro = self._write_read(data)
                    return await asyncio.wait_for(coro, self.timeout)
                except Exception as error:
                    self.log.error("write_read error: %r", error)
                    await self.close()

    # credo che questa funzione gestisca sia il read che il write ma non sono sicuro
    async def _write_read(self, data):
        await self._write(data)
        return await self._read()

    async def handle_client(self, reader, writer):
        async with Client(reader, writer) as client:  # creo una instance di un Client
            while True:  # while true gestisco le varie richieste che mi arrivano
                request = await client.read()
                if not request:
                    return
                reply = await self.write_read(request)
                if not reply:
                    return
                result = await client.write(reply)
                if not result:
                    return

    async def serve_forever(self):
        server = await asyncio.start_server(
            self.handle_client, self.host, self.port
        )  # creo un sever dove l'handle client è gestito dalla apposita funzione
        async with server:
            self.log.info("Ready to accept requests on %s:%d", self.host, self.port)
            await server.serve_forever()  # attivo il server


async def run(server_url, modbus_url, timeout, connection_time):
    async with ModBus(server_url.hostname,
                      server_url.port,
                      modbus_url.hostname,
                      modbus_url.port,
                      timeout,
                      connection_time) as modbus:  # creo una modbus connection
        await modbus.serve_forever()  # e la attivo per sempre


def load_log_config(file_name):
    global log
    if not file_name:
        logging.config.dictConfig(DEFAULT_LOG_CONFIG)
        return
    file_name = pathlib.Path(file_name)
    ext = file_name.suffix
    if ext.endswith('toml'):
        from toml import load
    elif ext.endswith('yml') or ext.endswith('yaml'):
        import yaml
        def load(fobj):
            return yaml.load(fobj, Loader=yaml.Loader)
    elif ext.endswith('json'):
        from json import load
    elif ext.endswith('ini') or ext.endswith('conf'):
        logging.config.fileConfig(file_name, disable_existing_loggers=False)
        return
    else:
        raise NotImplementedError
    with open(file_name) as fobj:
        obj = load(fobj)
    obj.setdefault("version", 1)
    obj.setdefault("disable_existing_loggers", False)
    logging.config.dictConfig(obj)
    log = logging.getLogger("modbus-proxy")


def main():

    import json

    driver_params = json.load(open("../configuration/TCS_driver_parameters.json"))

    controller_ip = driver_params["ip&ports"]["az_controller_ip"]
    controller_port = driver_params["ip&ports"]["az_controller_port"]

    proxy_ip = driver_params["ip&ports"]["az_proxy_ip"]
    proxy_port = driver_params["ip&ports"]["az_proxy_port"]

    full_ip_az_controller=urlparse("tcp://"+controller_ip+":"+str(controller_port))
    full_ip_az_proxy=urlparse("tcp://"+proxy_ip+":"+str(proxy_port))

    load_log_config(None)  # prende la configurazione dei login dal configuration file
    global log
    log = logging.getLogger("modbus-proxy")
    log.info("Starting...")

    try:
        # lancio l'asyncio.run program che dovrebbe fare tutto????
        asyncio.run(run(full_ip_az_proxy, full_ip_az_controller, timeout=10, connection_time=.1))
    except KeyboardInterrupt:
        log.warning("Ctrl-C pressed. Bailing out!")

if __name__ == "__main__":
    main()