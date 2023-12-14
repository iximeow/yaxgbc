
pub fn run(tx: std::sync::mpsc::Sender<()>) {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();

    rt.block_on(async move {
        loop {
            tokio::time::sleep(std::time::Duration::from_micros(16_666)).await;
            tx.send(()).expect("TODO: receiver is still running");
        }
    });
}
