use std::{
    io,
    thread,
    sync::mpsc,
};

use rppal::spi::{
    Spi,
    Bus,
    Mode,
    Segment,
    SlaveSelect,
};

use rpi_lfa::Volt;

pub enum Session {
    Initializing(Initializing),
    Ready(Ready),
    Probing(Probing),
}

#[derive(Clone, Debug)]
pub struct Params {
    pub voltage_drain: Vdd,
    pub voltage_ref: Vref,
}

#[derive(Clone, Debug)]
pub enum Vdd {
    Positive3v3,
    Positive5v,
}

#[derive(Clone, Debug)]
pub enum Vref {
    EqualToVdd,
    Other { voltage: Volt, },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Channel {
    Ch0,
    Ch1,
    Ch2,
    Ch3,
    Ch4,
    Ch5,
    Ch6,
    Ch7,
}

#[derive(Debug)]
pub enum Error {
    SpiThreadSpawn(io::Error),
    SpiThreadLost,
    SpiInitialize(rppal::spi::Error),
    SpiTransferSegments(rppal::spi::Error),
}

impl Session {
    pub fn new(params: &Params) -> Result<Self, Error> {
        let hz = match params.voltage_drain {
            Vdd::Positive3v3 =>
                1_350_000,
            Vdd::Positive5v =>
                3_600_000,
        };
        let v_ref = match params.voltage_ref {
            Vref::EqualToVdd =>
                match params.voltage_drain {
                    Vdd::Positive3v3 =>
                        Volt(3.3),
                    Vdd::Positive5v =>
                        Volt(5.0),
                },
            Vref::Other { voltage, } =>
                voltage,
        };

        let (request_tx, request_rx) = mpsc::sync_channel(0);
        let (event_tx, event_rx) = mpsc::sync_channel(0);

        let _builder = thread::Builder::new()
            .name("Mcp3008 spi".into())
            .spawn(move || spi_worker(request_rx, event_tx, hz, v_ref))
            .map_err(Error::SpiThreadSpawn)?;

        Ok(Session::Initializing(Initializing {
            inner: Inner { request_tx, event_rx, },
        }))
    }
}

// Initializing

pub struct Initializing {
    inner: Inner,
}

impl From<Initializing> for Session {
    fn from(state: Initializing) -> Session {
        Session::Initializing(state)
    }
}

impl Initializing {
    pub fn probe(self) -> Result<InitializingOp, Error> {
        match self.inner.event_rx.try_recv() {
            Ok(Event::SpiInitialized) =>
                Ok(InitializingOp::Ready(Ready { inner: self.inner, })),
            Ok(Event::ChannelRead { .. }) =>
                unreachable!(),
            Ok(Event::Error(error)) =>
                Err(error),
            Err(mpsc::TryRecvError::Empty) =>
                Ok(InitializingOp::Idle(self)),
            Err(mpsc::TryRecvError::Disconnected) =>
                Err(Error::SpiThreadLost),
        }
    }

}

pub enum InitializingOp {
    Idle(Initializing),
    Ready(Ready),
}

// Ready

pub struct Ready {
    inner: Inner,
}

impl From<Ready> for Session {
    fn from(state: Ready) -> Session {
        Session::Ready(state)
    }
}

impl Ready {
    pub fn probe_channel(self, channel: Channel) -> Probing {
        Probing {
            state: ProbingState::Request { channel, },
            inner: self.inner,
        }
    }
}

// Probing

pub struct Probing {
    state: ProbingState,
    inner: Inner,
}

impl From<Probing> for Session {
    fn from(state: Probing) -> Session {
        Session::Probing(state)
    }
}

enum ProbingState {
    Request { channel: Channel, },
    WaitingReply,
}

impl Probing {
    pub fn poll(mut self) -> Result<ProbingOp, Error> {
        loop {
            match self.state {
                ProbingState::Request { channel, } =>
                    match self.inner.request_tx.try_send(Request::ProbeChannel { channel, }) {
                        Ok(()) =>
                            self.state = ProbingState::WaitingReply,
                        Err(mpsc::TrySendError::Full(..)) =>
                            return Ok(ProbingOp::Idle(self)),
                        Err(mpsc::TrySendError::Disconnected(..)) =>
                            return Err(Error::SpiThreadLost),
                    },
                ProbingState::WaitingReply =>
                    match self.inner.event_rx.try_recv() {
                        Ok(Event::SpiInitialized) =>
                            unreachable!(),
                        Ok(Event::ChannelRead { channel, value, }) =>
                            return Ok(ProbingOp::Done {
                                channel,
                                value,
                                ready: Ready { inner: self.inner, },
                            }),
                        Ok(Event::Error(error)) =>
                            return Err(error),
                        Err(mpsc::TryRecvError::Empty) =>
                            return Ok(ProbingOp::Idle(self)),
                        Err(mpsc::TryRecvError::Disconnected) =>
                            return Err(Error::SpiThreadLost),
                    }
            }
        }
    }
}

pub enum ProbingOp {
    Idle(Probing),
    Done {
        channel: Channel,
        value: Volt,
        ready: Ready,
    },
}

// inner impl

struct Inner {
    request_tx: mpsc::SyncSender<Request>,
    event_rx: mpsc::Receiver<Event>,
}

enum Request {
    ProbeChannel { channel: Channel, },
}

enum Event {
    SpiInitialized,
    ChannelRead { channel: Channel, value: Volt, },
    Error(Error),
}

fn spi_worker(request_rx: mpsc::Receiver<Request>, event_tx: mpsc::SyncSender<Event>, hz: u32, v_ref: Volt) {
    if let Err(error) = spi_worker_loop(request_rx, &event_tx, hz, v_ref) {
        event_tx.send(Event::Error(error)).ok();
    }
}

fn spi_worker_loop(
    request_rx: mpsc::Receiver<Request>,
    event_tx: &mpsc::SyncSender<Event>,
    hz: u32,
    v_ref: Volt,
)
    -> Result<(), Error>
{
    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, hz, Mode::Mode0)
        .map_err(Error::SpiInitialize)?;
    let mut buffer: [u8; 3] = [0, 0, 0];

    event_tx.send(Event::SpiInitialized)
        .or_else(|mpsc::SendError(..)| Ok(()))?;

    loop {
        match request_rx.recv() {
            Ok(Request::ProbeChannel { channel, }) => {
                let channel_value = match channel {
                    Channel::Ch0 => 0,
                    Channel::Ch1 => 1,
                    Channel::Ch2 => 2,
                    Channel::Ch3 => 3,
                    Channel::Ch4 => 4,
                    Channel::Ch5 => 5,
                    Channel::Ch6 => 6,
                    Channel::Ch7 => 7,
                };
                spi.transfer_segments(
                    &[Segment::new(&mut buffer, &[0b00000001, 0b10000000 | (channel_value << 4), 0b00000000])],
                ).map_err(Error::SpiTransferSegments)?;
                let data = ((buffer[1] & 0b00000011) as u16) << 8 | (buffer[2] as u16);
                let value = Volt(data as f64 * v_ref.0 / 1024.0);

                event_tx.send(Event::ChannelRead { channel, value, })
                    .or_else(|mpsc::SendError(..)| Ok(()))?;
            },
            Err(mpsc::RecvError) =>
                return Ok(()),
        }
    }
}
