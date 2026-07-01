use thiserror::Error;

#[derive(Debug, Error)]
pub enum SimError {
    #[error("terminal I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("invalid scenario configuration: {0}")]
    Scenario(String),
}

pub type SimResult<T> = Result<T, SimError>;
